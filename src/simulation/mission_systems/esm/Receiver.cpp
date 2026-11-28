#include "simulation/mission_systems/esm/Receiver.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace fsim::ms::esm {

namespace {

constexpr double kPi = geom::kPi;
constexpr double kDeg = kPi / 180.0;

// What a card that says nothing of its receiver flies with - no maker publishes these for a type in service. One
// class for every country's, engineering assumptions each (a card's own figure, where one is found, replaces it):
constexpr double kSensitivityDbm = -60.0;   // the weakest pulse at its antenna it detects (a wide-open superheterodyne's class)
constexpr double kAntennaGainDbi = 0.0;     // a cavity-backed spiral's, as broad as a quadrant
constexpr double kBandMinHz = 2e9;          // 2 to 18 GHz: the band warning receivers are described as covering
constexpr double kBandMaxHz = 18e9;
constexpr double kBearingSigmaDeg = 10.0;   // amplitude comparison between four antennas
constexpr double kMeasureS = 1.0;           // a bearing is measured anew this often
constexpr double kMemoryS = 3.0;            // an emitter no longer heard is kept this long
constexpr std::size_t kContacts = 64;
constexpr std::size_t kGeoSamplesMax = 64;

// Generic wide-open superheterodyne RWR fallback when a synthetic card omits parameters;
// all maintained esm cards declare their own parameters in assets/payloads/*.json.
struct EsmClassDefaults {
    double sensitivityDbm = kSensitivityDbm;
    double antennaGainDbi = kAntennaGainDbi;
    double bandMinHz = kBandMinHz;
    double bandMaxHz = kBandMaxHz;
    double bearingSigmaDeg = kBearingSigmaDeg;
    double bearingFloorDeg = kBearingSigmaDeg;
    bool geolocation = false;
    bool comint = false;
};

void ecefToGeodetic(const double p[3], double& latRad, double& lonRad, double& heightM) noexcept {
    constexpr double a = 6378137.0;
    constexpr double e2 = 0.00669437999014;
    const double horiz = std::hypot(p[0], p[1]);
    double lat = std::atan2(p[2], horiz * (1.0 - e2));
    double n = a;
    for (int k = 0; k < 5; ++k) {
        const double sl = std::sin(lat);
        n = a / std::sqrt(1.0 - e2 * sl * sl);
        lat = std::atan2(p[2] + e2 * n * sl, horiz);
    }
    latRad = lat;
    lonRad = std::atan2(p[1], p[0]);
    heightM = horiz / std::max(std::cos(lat), 1e-9) - n;
}

bool invert3x3(const double m[3][3], double inv[3][3]) noexcept {
    const double det =
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
        m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
        m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    const double trace = m[0][0] + m[1][1] + m[2][2];
    if (!std::isfinite(det) || !(trace > 1e-18) || det <= 1e-11 * trace * trace * trace) return false;
    const double invDet = 1.0 / det;
    inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet;
    inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invDet;
    inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet;
    inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invDet;
    inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet;
    inv[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * invDet;
    inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet;
    inv[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * invDet;
    inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invDet;
    return true;
}

double mlAngularCost(const BearingSample* samples, std::size_t count, const double ownPos[3], const double dp[3]) noexcept {
    double cost = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
        const double invVar = 1.0 / std::max(samples[i].sigmaRad * samples[i].sigmaRad, 1e-8);
        const double d[3] = {
            dp[0] - (samples[i].originEcef[0] - ownPos[0]),
            dp[1] - (samples[i].originEcef[1] - ownPos[1]),
            dp[2] - (samples[i].originEcef[2] - ownPos[2])};
        const double r = geom::norm3(d);
        if (!(r > 1.0)) return std::numeric_limits<double>::infinity();
        const double ex = d[0] / r - samples[i].rayEcef[0];
        const double ey = d[1] / r - samples[i].rayEcef[1];
        const double ez = d[2] / r - samples[i].rayEcef[2];
        cost += 0.5 * invVar * (ex * ex + ey * ey + ez * ez);
    }
    return cost;
}

} // namespace

TriangulationResult triangulateBearings(const BearingSample* samples, std::size_t count,
                                        const double ownPos[3], const double latestRay[3]) noexcept {
    TriangulationResult out;
    if (!samples || count < 4) return out;

    double maxBaselineSq = 0.0;
    double minCosParallax = 1.0;
    const auto& first = samples[0];
    for (std::size_t i = 1; i < count; ++i) {
        const double dx = samples[i].originEcef[0] - first.originEcef[0];
        const double dy = samples[i].originEcef[1] - first.originEcef[1];
        const double dz = samples[i].originEcef[2] - first.originEcef[2];
        maxBaselineSq = std::max(maxBaselineSq, dx * dx + dy * dy + dz * dz);
        const double c = samples[i].rayEcef[0] * first.rayEcef[0] +
                         samples[i].rayEcef[1] * first.rayEcef[1] +
                         samples[i].rayEcef[2] * first.rayEcef[2];
        minCosParallax = std::min(minCosParallax, std::clamp(c, -1.0, 1.0));
    }
    // Require >= 2 km baseline and >= 4 deg parallax before publishing a fix.
    if (maxBaselineSq < 2000.0 * 2000.0) return out;
    if (std::acos(minCosParallax) < 4.0 * kDeg) return out;

    // Stage 1: Weighted orthogonal-projection (Stansfield-class) linear initializer
    // in coordinates shifted relative to `ownPos`.
    double A[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    double b[3] = {0, 0, 0};
    for (std::size_t i = 0; i < count; ++i) {
        const double w = 1.0 / std::max(samples[i].sigmaRad * samples[i].sigmaRad, 1e-8);
        const double o[3] = {
            samples[i].originEcef[0] - ownPos[0],
            samples[i].originEcef[1] - ownPos[1],
            samples[i].originEcef[2] - ownPos[2]};
        const double u[3] = {samples[i].rayEcef[0], samples[i].rayEcef[1], samples[i].rayEcef[2]};
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                const double m = ((r == c) ? 1.0 : 0.0) - u[r] * u[c];
                A[r][c] += w * m;
                b[r] += w * m * o[c];
            }
        }
    }

    double invA[3][3];
    if (!invert3x3(A, invA)) return out;

    double dp[3] = {0, 0, 0};
    for (int r = 0; r < 3; ++r) {
        dp[r] = invA[r][0] * b[0] + invA[r][1] * b[1] + invA[r][2] * b[2];
        if (!std::isfinite(dp[r])) return out;
    }
    const double initRange = geom::norm3(dp);
    if (!(initRange >= 500.0 && initRange <= 600000.0)) return out;
    if (geom::dot3(dp, latestRay) <= 0.0) return out;

    // Stage 2: Damped Gauss-Newton angular Maximum Likelihood refinement
    // minimizing 0.5 * sum_k (1/sigma_k^2) || (x - o_k)/||x - o_k|| - u_k ||^2.
    double cost = mlAngularCost(samples, count, ownPos, dp);
    for (int iter = 0; iter < 6; ++iter) {
        double fim[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
        double grad[3] = {0, 0, 0};
        for (std::size_t i = 0; i < count; ++i) {
            const double invVar = 1.0 / std::max(samples[i].sigmaRad * samples[i].sigmaRad, 1e-8);
            const double d[3] = {
                dp[0] - (samples[i].originEcef[0] - ownPos[0]),
                dp[1] - (samples[i].originEcef[1] - ownPos[1]),
                dp[2] - (samples[i].originEcef[2] - ownPos[2])};
            const double r = geom::norm3(d);
            if (!(r > 1.0)) break;
            const double dh[3] = {d[0] / r, d[1] / r, d[2] / r};
            const double u[3] = {samples[i].rayEcef[0], samples[i].rayEcef[1], samples[i].rayEcef[2]};
            const double wFim = invVar / (r * r);
            const double wGrad = invVar / r;
            const double dotDu = geom::dot3(dh, u);
            for (int row = 0; row < 3; ++row) {
                grad[row] += -wGrad * (u[row] - dotDu * dh[row]);
                for (int col = 0; col < 3; ++col) {
                    fim[row][col] += wFim * (((row == col) ? 1.0 : 0.0) - dh[row] * dh[col]);
                }
            }
        }
        const double tr = fim[0][0] + fim[1][1] + fim[2][2];
        if (!(tr > 0.0)) break;
        const double damp = 1e-6 * tr;
        for (int k = 0; k < 3; ++k) fim[k][k] += damp;
        double invF[3][3];
        if (!invert3x3(fim, invF)) break;
        double step[3] = {0, 0, 0};
        for (int row = 0; row < 3; ++row) {
            step[row] = -(invF[row][0] * grad[0] + invF[row][1] * grad[1] + invF[row][2] * grad[2]);
        }
        if (geom::norm3(step) < 0.5) break;

        bool accepted = false;
        for (double alpha = 1.0; alpha >= 0.125; alpha *= 0.5) {
            const double cand[3] = {dp[0] + alpha * step[0], dp[1] + alpha * step[1], dp[2] + alpha * step[2]};
            const double candRange = geom::norm3(cand);
            if (!(candRange >= 500.0 && candRange <= 600000.0)) continue;
            if (geom::dot3(cand, latestRay) <= 0.0) continue;
            const double candCost = mlAngularCost(samples, count, ownPos, cand);
            if (candCost < cost) {
                for (int k = 0; k < 3; ++k) dp[k] = cand[k];
                cost = candCost;
                accepted = true;
                break;
            }
        }
        if (!accepted) break;
    }

    for (int k = 0; k < 3; ++k) out.positionEcef[k] = ownPos[k] + dp[k];
    out.valid = true;

    // Stage 3: Exact 3-D Fisher Information Matrix & Cramér-Rao Lower Bound (CRLB)
    // covariance C = F(x*)^-1, plus horizontal (East-North) elliptical CEP50.
    double fim[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    for (std::size_t i = 0; i < count; ++i) {
        const double invVar = 1.0 / std::max(samples[i].sigmaRad * samples[i].sigmaRad, 1e-8);
        const double d[3] = {
            dp[0] - (samples[i].originEcef[0] - ownPos[0]),
            dp[1] - (samples[i].originEcef[1] - ownPos[1]),
            dp[2] - (samples[i].originEcef[2] - ownPos[2])};
        const double r = geom::norm3(d);
        if (!(r > 1.0)) continue;
        const double dh[3] = {d[0] / r, d[1] / r, d[2] / r};
        const double w = invVar / (r * r);
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                fim[row][col] += w * (((row == col) ? 1.0 : 0.0) - dh[row] * dh[col]);
            }
        }
    }
    double cov[3][3];
    if (invert3x3(fim, cov)) {
        const double trCov = cov[0][0] + cov[1][1] + cov[2][2];
        if (trCov > 0.0 && std::isfinite(trCov)) {
            out.crlbRmsM = std::sqrt(trCov);
            // Project 3x3 ECEF covariance onto local horizontal (East, North) plane at estEcef.
            double lat = 0.0, lon = 0.0, hM = 0.0;
            ecefToGeodetic(out.positionEcef, lat, lon, hM);
            const double sl = std::sin(lat), cl = std::cos(lat);
            const double so = std::sin(lon), co = std::cos(lon);
            const double east[3] = {-so, co, 0.0};
            const double north[3] = {-sl * co, -sl * so, cl};
            auto quad = [&](const double u[3], const double v[3]) noexcept {
                double s = 0.0;
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c) s += u[r] * cov[r][c] * v[c];
                return s;
            };
            const double cEE = std::max(0.0, quad(east, east));
            const double cNN = std::max(0.0, quad(north, north));
            const double cEN = quad(east, north);
            const double mean = 0.5 * (cEE + cNN);
            const double diff = 0.5 * (cEE - cNN);
            const double rad = std::hypot(diff, cEN);
            const double sigMax = std::sqrt(std::max(0.0, mean + rad));
            const double sigMin = std::sqrt(std::max(0.0, mean - rad));
            // Standard elliptical CEP50 approximation (O'Donoughue / Grubbs): 0.5887 * (sigma_a + sigma_b).
            out.crlbCep50M = 0.5887 * (sigMax + sigMin);
        }
    }
    return out;
}

namespace {

class Receiver final : public Model {
public:
    explicit Receiver(const Card& card) {
        const EsmClassDefaults def{};
        sensitivityDbm_ = card.value("sensitivity_dbm", def.sensitivityDbm);
        antennaGainDbi_ = card.value("antenna_gain_dbi", def.antennaGainDbi);
        bandMinHz_ = card.value("frequency_min_hz", def.bandMinHz);
        bandMaxHz_ = card.value("frequency_max_hz", def.bandMaxHz);
        bearingSigmaRad_ = card.value("bearing_sigma_deg", def.bearingSigmaDeg) * kDeg;
        bearingFloorRad_ = std::min(bearingSigmaRad_, card.value("bearing_floor_deg", def.bearingFloorDeg) * kDeg);
        measureS_ = card.value("measure_s", kMeasureS);
        memoryS_ = card.value("memory_s", kMemoryS);
        geolocation_ = card.value("geolocation", def.geolocation ? 1.0 : 0.0) > 0.5;
        comint_ = card.value("comint", def.comint ? 1.0 : 0.0) > 0.5;
    }

    void step(double now, const sim::VehicleState& own, const Scene& scene, Rng& rng) override {
        for (const Emission& e : scene.emissions()) {
            if (!e.state || e.state == &own) continue; // (its own carrier's transmitters: blanked)
            if (e.frequencyHz < bandMinHz_ || e.frequencyHz > bandMaxHz_) continue;
            if (e.mode == EmitterMode::Datalink && !comint_) continue;
            double los[3];
            if (!(geom::lineOfSight(own, e.state->positionEcef, los) > 0.0)) continue;
            const double dbm = receivedDbm(e, own, scene.pathLossDb(e.entity, e.frequencyHz), antennaGainDbi_);
            if (!(dbm >= sensitivityDbm_)) continue;
            Held* h = held(e, now);
            if (!h) continue;

            // SNR-dependent DF standard deviation: at sensitivity threshold equals
            // bearingSigmaRad_, tightening with excess SNR down to bearingFloorRad_.
            const double excessDb = std::max(0.0, dbm - sensitivityDbm_);
            const double snrFactor = std::sqrt(std::pow(10.0, excessDb / 10.0));
            const double sigmaEff = std::clamp(bearingSigmaRad_ / std::max(1.0, snrFactor),
                                               bearingFloorRad_, bearingSigmaRad_);

            const bool newMeasurement = (now - h->measuredS >= measureS_);
            if (newMeasurement) { // a new measurement: a new error
                h->measuredS = now;
                h->sigmaEff = sigmaEff;
                h->azimuthError = sigmaEff * rng.normal();
                h->elevationError = sigmaEff * rng.normal();
            }

            double body[3], azimuth = 0.0, elevation = 0.0;
            geom::toBody(own, los, body);
            geom::angles(body, azimuth, elevation);
            Contact& c = h->contact;
            c.timeS = now;
            c.azimuthRad = std::remainder(azimuth + h->azimuthError, 2.0 * kPi);
            c.elevationRad = std::clamp(elevation + h->elevationError, -0.5 * kPi, 0.5 * kPi);
            c.bearingSigmaRad = h->sigmaEff;
            c.strengthDb = dbm;
            c.frequencyHz = e.frequencyHz;

            // What the emitter does, as its signal says: a beam that stays on the carrier is a lock (and, lighting it,
            // an illumination); one that passes as it scans, or a side lobe of one pointed elsewhere, is a search.
            const bool mainBeam = e.gainTowardsDbi(own.positionEcef) > 0.5 * (e.peakGainDbi + e.sidelobeDbi);
            switch (e.mode) {
            case EmitterMode::Lock:
            case EmitterMode::Illumination: c.mode = mainBeam ? e.mode : EmitterMode::Search; break;
            default: c.mode = e.mode; break;
            }
            c.warning = mainBeam && (c.mode == EmitterMode::Lock || c.mode == EmitterMode::Illumination || c.mode == EmitterMode::Seeker);

            // Moving-platform bearing-only triangulation (geolocation) for capable ESM suites.
            if (geolocation_ && newMeasurement) {
                const double mBody[3] = {
                    std::cos(c.elevationRad) * std::cos(c.azimuthRad),
                    std::cos(c.elevationRad) * std::sin(c.azimuthRad),
                    -std::sin(c.elevationRad)};
                double mRay[3];
                geom::toEcef(own, mBody, mRay);
                const double rayNorm = geom::norm3(mRay);
                if (rayNorm > 1e-9) {
                    for (double& v : mRay) v /= rayNorm;
                    bool farEnough = h->geoSamples.empty();
                    if (!farEnough) {
                        const auto& prev = h->geoSamples.back().originEcef;
                        const double dx = own.positionEcef[0] - prev[0];
                        const double dy = own.positionEcef[1] - prev[1];
                        const double dz = own.positionEcef[2] - prev[2];
                        farEnough = (dx * dx + dy * dy + dz * dz >= 400.0 * 400.0);
                    }
                    if (farEnough) {
                        if (h->geoSamples.size() >= kGeoSamplesMax) {
                            h->geoSamples.erase(h->geoSamples.begin());
                        }
                        BearingSample sample;
                        for (int k = 0; k < 3; ++k) {
                            sample.originEcef[k] = own.positionEcef[k];
                            sample.rayEcef[k] = mRay[k];
                        }
                        sample.sigmaRad = std::max(h->sigmaEff, 1e-4);
                        h->geoSamples.push_back(sample);
                    }
                    const TriangulationResult geo = triangulateBearings(
                        h->geoSamples.data(), h->geoSamples.size(), own.positionEcef, mRay);
                    if (geo.valid) {
                        double lat = 0.0, lon = 0.0, hM = 0.0;
                        ecefToGeodetic(geo.positionEcef, lat, lon, hM);
                        const double rel[3] = {geo.positionEcef[0] - own.positionEcef[0],
                                               geo.positionEcef[1] - own.positionEcef[1],
                                               geo.positionEcef[2] - own.positionEcef[2]};
                        c.rangeM = geom::norm3(rel);
                        c.latitudeRad = lat;
                        c.longitudeRad = lon;
                        c.heightM = hM;
                        if (std::isfinite(geo.crlbCep50M)) {
                            c.sizeM = geo.crlbCep50M;
                        }
                    }
                }
            }
        }
        held_.erase(std::remove_if(held_.begin(), held_.end(), [&](const Held& h) { return now - h.contact.timeS > memoryS_; }), held_.end());
    }

    void idle(double) override { held_.clear(); }

    void contacts(std::vector<Contact>& out) const override {
        for (const Held& h : held_) out.push_back(h.contact);
    }

    void report(std::vector<PayloadValue>& out) const override {
        double warnings = 0.0;
        double jammers = 0.0;
        double comintContacts = 0.0;
        double geolocated = 0.0;
        double minCepM = std::numeric_limits<double>::infinity();
        for (const Held& h : held_) {
            warnings += h.contact.warning ? 1.0 : 0.0;
            jammers += (h.contact.mode == EmitterMode::Jammer) ? 1.0 : 0.0;
            comintContacts += (h.contact.mode == EmitterMode::Datalink) ? 1.0 : 0.0;
            if (std::isfinite(h.contact.rangeM)) {
                geolocated += 1.0;
                if (std::isfinite(h.contact.sizeM)) {
                    minCepM = std::min(minCepM, h.contact.sizeM);
                }
            }
        }
        out.push_back({"contacts", static_cast<double>(held_.size()), ""});
        out.push_back({"warnings", warnings, ""});
        out.push_back({"jammers", jammers, ""});
        out.push_back({"comint_contacts", comintContacts, ""});
        out.push_back({"geolocated", geolocated, ""});
        out.push_back({"geo_cep_min", std::isfinite(minCepM) ? minCepM : std::numeric_limits<double>::quiet_NaN(), "m"});
        out.push_back({"sensitivity", sensitivityDbm_, "dBm"});
    }

private:
    struct Held {
        Contact contact;
        std::uint32_t entity = 0;
        std::uint16_t payload = 0; ///< the emitter's own, on its carrier
        double measuredS = -std::numeric_limits<double>::infinity();
        double sigmaEff = 0.0;
        double azimuthError = 0.0, elevationError = 0.0;
        std::vector<BearingSample> geoSamples;
    };

    Held* held(const Emission& e, double now) {
        for (Held& h : held_)
            if (h.entity == e.entity && h.payload == e.payload && h.contact.mode == e.mode) return &h;
        // Also match same entity+payload when radar transitions Search <-> Lock <-> Illumination.
        for (Held& h : held_)
            if (h.entity == e.entity && h.payload == e.payload &&
                e.mode != EmitterMode::Datalink && h.contact.mode != EmitterMode::Datalink) return &h;
        if (held_.size() >= kContacts) return nullptr;
        Held h;
        h.entity = e.entity, h.payload = e.payload;
        h.sigmaEff = bearingSigmaRad_;
        h.contact.id = ++serial_;
        h.contact.kind = ContactKind::Emitter;
        h.contact.firstTimeS = now;
        h.contact.truth = e.entity;
        held_.push_back(h);
        return &held_.back();
    }

    double sensitivityDbm_ = kSensitivityDbm;
    double antennaGainDbi_ = kAntennaGainDbi;
    double bandMinHz_ = kBandMinHz;
    double bandMaxHz_ = kBandMaxHz;
    double bearingSigmaRad_ = kBearingSigmaDeg * kDeg;
    double bearingFloorRad_ = 3.0 * kDeg;
    double measureS_ = kMeasureS;
    double memoryS_ = kMemoryS;
    bool geolocation_ = false;
    bool comint_ = false;
    std::vector<Held> held_;
    std::uint64_t serial_ = 0;
};

} // namespace

double receivedDbm(const Emission& emission, const sim::VehicleState& own, double pathLossDb, double antennaGainDbi) noexcept {
    const double watts = emission.peakPowerW > 0.0 ? emission.peakPowerW : emission.powerW;
    return 10.0 * std::log10(std::max(watts, 1e-12) * 1000.0) + emission.gainTowardsDbi(own.positionEcef) - pathLossDb + antennaGainDbi;
}

std::unique_ptr<Model> makeReceiver(const Card& card) { return std::make_unique<Receiver>(card); }

} // namespace fsim::ms::esm
