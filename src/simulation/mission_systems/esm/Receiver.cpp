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

EsmClassDefaults esmDefaultsFor(const Card& card) {
    EsmClassDefaults d;
    if (card.id == "rivet-joint") {
        // Dedicated airborne SIGINT (COMINT + ELINT) suite (basis: assumed class bounds).
        d.sensitivityDbm = -85.0;
        d.antennaGainDbi = 3.0;
        d.bandMinHz = 0.03e9; // VHF/UHF COMINT through X/Ku ELINT
        d.bandMaxHz = 18.0e9;
        d.bearingSigmaDeg = 1.5;
        d.bearingFloorDeg = 0.5;
        d.geolocation = true;
        d.comint = true;
    } else if (card.id == "an-alq-218" || card.id == "e3-esm" || card.id == "e7-esm") {
        // Wideband interferometric ESM/ELINT suites capable of DF geolocation and L-band link intercept.
        d.sensitivityDbm = -75.0;
        d.antennaGainDbi = 2.0;
        d.bandMinHz = 0.5e9;
        d.bandMaxHz = 18.0e9;
        d.bearingSigmaDeg = 2.0;
        d.bearingFloorDeg = 0.6;
        d.geolocation = true;
        d.comint = true;
    } else if (card.id == "an-alr-94" || card.id == "an-asq-239" || card.id == "spectra" ||
               card.id == "praetorian" || card.id == "gripen-ews") {
        // Integrated fighter EW/ESM suites with interferometric DF and moving-baseline geolocation.
        d.sensitivityDbm = -70.0;
        d.antennaGainDbi = 1.0;
        d.bandMinHz = 0.5e9;
        d.bandMaxHz = 18.0e9;
        d.bearingSigmaDeg = 3.0;
        d.bearingFloorDeg = 1.0;
        d.geolocation = true;
        d.comint = false;
    }
    return d;
}

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

struct GeoSample {
    std::array<double, 3> origin;
    std::array<double, 3> ray;
    double weight = 1.0;
};

// Deterministic 3-D bearing-only least-squares triangulation:
// minimizes sum_k w_k || (I - u_k u_k^T)(p - o_k) ||^2.
// Returns true and populates `estEcef` only when baseline >= 2 km, angular
// parallax >= 4 deg, the normal matrix is well-conditioned, and the fix lies
// in front of the receiver within [500 m, 600 km].
bool triangulateRays(const std::vector<GeoSample>& samples, const double ownPos[3],
                     const double latestRay[3], double estEcef[3]) noexcept {
    if (samples.size() < 4) return false;

    double maxBaselineSq = 0.0;
    double minCosParallax = 1.0;
    const auto& first = samples.front();
    for (std::size_t i = 1; i < samples.size(); ++i) {
        const double dx = samples[i].origin[0] - first.origin[0];
        const double dy = samples[i].origin[1] - first.origin[1];
        const double dz = samples[i].origin[2] - first.origin[2];
        maxBaselineSq = std::max(maxBaselineSq, dx * dx + dy * dy + dz * dz);
        const double c = samples[i].ray[0] * first.ray[0] +
                         samples[i].ray[1] * first.ray[1] +
                         samples[i].ray[2] * first.ray[2];
        minCosParallax = std::min(minCosParallax, std::clamp(c, -1.0, 1.0));
    }
    // Require >= 2 km baseline and >= 4 deg parallax before publishing a fix.
    if (maxBaselineSq < 2000.0 * 2000.0) return false;
    if (std::acos(minCosParallax) < 4.0 * kDeg) return false;

    // Work in coordinates shifted relative to `ownPos` for numerical conditioning.
    double A[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    double b[3] = {0, 0, 0};
    for (const auto& s : samples) {
        const double w = s.weight;
        const double o[3] = {s.origin[0] - ownPos[0], s.origin[1] - ownPos[1], s.origin[2] - ownPos[2]};
        const double u[3] = {s.ray[0], s.ray[1], s.ray[2]};
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                const double m = ((r == c) ? 1.0 : 0.0) - u[r] * u[c];
                A[r][c] += w * m;
                b[r] += w * m * o[c];
            }
        }
    }

    const double det =
        A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
        A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
        A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    const double trace = A[0][0] + A[1][1] + A[2][2];
    if (!std::isfinite(det) || !(trace > 1e-12) || det <= 1e-9 * trace * trace * trace) return false;

    const double invDet = 1.0 / det;
    double inv[3][3];
    inv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * invDet;
    inv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * invDet;
    inv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invDet;
    inv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * invDet;
    inv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invDet;
    inv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * invDet;
    inv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * invDet;
    inv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * invDet;
    inv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * invDet;

    double dp[3] = {0, 0, 0};
    for (int r = 0; r < 3; ++r) {
        dp[r] = inv[r][0] * b[0] + inv[r][1] * b[1] + inv[r][2] * b[2];
        if (!std::isfinite(dp[r])) return false;
    }
    const double range = geom::norm3(dp);
    if (!(range >= 500.0 && range <= 600000.0)) return false;
    // Must lie in front of the latest measured line of sight.
    if (geom::dot3(dp, latestRay) <= 0.0) return false;

    for (int k = 0; k < 3; ++k) estEcef[k] = ownPos[k] + dp[k];
    return true;
}

class Receiver final : public Model {
public:
    explicit Receiver(const Card& card) {
        const EsmClassDefaults def = esmDefaultsFor(card);
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
                        const auto& prev = h->geoSamples.back().origin;
                        const double dx = own.positionEcef[0] - prev[0];
                        const double dy = own.positionEcef[1] - prev[1];
                        const double dz = own.positionEcef[2] - prev[2];
                        farEnough = (dx * dx + dy * dy + dz * dz >= 400.0 * 400.0);
                    }
                    if (farEnough) {
                        if (h->geoSamples.size() >= kGeoSamplesMax) {
                            h->geoSamples.erase(h->geoSamples.begin());
                        }
                        const double invVar = 1.0 / std::max(h->sigmaEff * h->sigmaEff, 1e-6);
                        h->geoSamples.push_back({
                            {own.positionEcef[0], own.positionEcef[1], own.positionEcef[2]},
                            {mRay[0], mRay[1], mRay[2]},
                            invVar});
                    }
                    double estEcef[3];
                    if (triangulateRays(h->geoSamples, own.positionEcef, mRay, estEcef)) {
                        double lat = 0.0, lon = 0.0, hM = 0.0;
                        ecefToGeodetic(estEcef, lat, lon, hM);
                        const double rel[3] = {estEcef[0] - own.positionEcef[0],
                                               estEcef[1] - own.positionEcef[1],
                                               estEcef[2] - own.positionEcef[2]};
                        c.rangeM = geom::norm3(rel);
                        c.latitudeRad = lat;
                        c.longitudeRad = lon;
                        c.heightM = hM;
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
        for (const Held& h : held_) {
            warnings += h.contact.warning ? 1.0 : 0.0;
            jammers += (h.contact.mode == EmitterMode::Jammer) ? 1.0 : 0.0;
            comintContacts += (h.contact.mode == EmitterMode::Datalink) ? 1.0 : 0.0;
            geolocated += std::isfinite(h.contact.rangeM) ? 1.0 : 0.0;
        }
        out.push_back({"contacts", static_cast<double>(held_.size()), ""});
        out.push_back({"warnings", warnings, ""});
        out.push_back({"jammers", jammers, ""});
        out.push_back({"comint_contacts", comintContacts, ""});
        out.push_back({"geolocated", geolocated, ""});
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
        std::vector<GeoSample> geoSamples;
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
