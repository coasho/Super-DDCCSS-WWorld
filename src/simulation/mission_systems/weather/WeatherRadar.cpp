#include "simulation/mission_systems/weather/WeatherRadar.h"
#include "simulation/mission_systems/ea/Jammer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fsim::ms::weather {

namespace {

constexpr double kPi = geom::kPi;
constexpr double kDeg = kPi / 180.0;
constexpr double kBoltzmannT0 = 1.380649e-23 * 290.0;
constexpr double kEffEarthRadiusM = (4.0 / 3.0) * 6371000.0; // Doviak & Zrnic (1993) Eq. 2.28 standard 4/3 Earth refraction

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

/// Deterministic geographically anchored 2-D mesoscale convective/stratiform modulation field
/// around `baseRainRateMmH`, producing distinct storm cores (~18-30 km) and clearer gaps while
/// preserving global mean intensity and `EnvironmentState` compatibility.
double spatialConvectiveModulation(double latRad, double lonRad) noexcept {
    constexpr double kEarthR = 6371000.0;
    const double yKm = latRad * kEarthR * 1e-3;
    const double xKm = lonRad * kEarthR * std::max(0.2, std::cos(latRad)) * 1e-3;
    // Multi-scale smooth harmonic storm field (22 km convective cores + 55 km rainband envelope)
    const double c1 = std::sin(2.0 * kPi * (xKm / 26.0 + 0.17 * yKm / 26.0)) *
                      std::cos(2.0 * kPi * (yKm / 22.0 - 0.23 * xKm / 22.0));
    const double c2 = std::cos(2.0 * kPi * (xKm / 55.0 - yKm / 48.0) + 0.7);
    const double raw = 1.0 + 0.65 * c1 + 0.35 * c2; // spans ~[0.0, 2.0], geographic mean = 1.0
    // Emphasize convective storm cores (> 1.0) and lighter stratiform gaps without creating false rain when base == 0
    return std::clamp(0.25 + 0.95 * std::max(0.0, raw), 0.25, 2.15);
}

class WeatherRadar final : public Model {
public:
    explicit WeatherRadar(const Card& card) {
        frequencyHz_ = card.value("frequency_hz", 9.375e9);
        powerW_ = card.value("power_w", 120.0);
        peakPowerW_ = card.value("peak_power_w", 6000.0);
        peakGainDbi_ = card.value("peak_gain_dbi", 30.0);
        azLimitRad_ = card.value("azimuth_limit_deg", 60.0) * kDeg;
        elLimitRad_ = card.value("elevation_limit_deg", 15.0) * kDeg;
        beamwidthRad_ = card.value("beamwidth_deg", 3.5) * kDeg;
        maxRangeM_ = card.value("range_max_m", 320000.0);
        // Probert-Jones minimum detectable reflectivity at 10 km reference (basis: assumed airborne X-band)
        mdsDbzAt10Km_ = card.value("mds_dbz_10km", 5.0);
    }

    Reason begin(const PayloadTask& task, double now) override {
        if (task.kind != TaskKind::Search) return Reason::InvalidParameter;
        if (!std::isfinite(task.azimuthCentreRad) || !std::isfinite(task.elevationCentreRad)) {
            return Reason::InvalidParameter;
        }
        if (std::isfinite(task.azimuthHalfWidthRad) && !(task.azimuthHalfWidthRad > 0.0)) return Reason::InvalidParameter;
        if (std::isfinite(task.elevationHalfWidthRad) && !(task.elevationHalfWidthRad > 0.0)) return Reason::InvalidParameter;
        if (std::isfinite(task.rangeScaleM) && !(task.rangeScaleM > 0.0)) return Reason::InvalidParameter;
        if (std::isfinite(task.durationS) && !(task.durationS > 0.0)) return Reason::InvalidParameter;

        const double azC = std::remainder(task.azimuthCentreRad, 2.0 * kPi);
        const double elC = std::clamp(task.elevationCentreRad, -0.5 * kPi, 0.5 * kPi);
        if (std::fabs(azC) > azLimitRad_ + 1e-6 || std::fabs(elC) > elLimitRad_ + 1e-6) {
            return Reason::OutOfReach;
        }

        searching_ = true;
        finished_ = Reason::None;
        startS_ = now;
        lastS_ = now;
        durationS_ = std::isfinite(task.durationS) ? task.durationS : std::numeric_limits<double>::quiet_NaN();
        azCentreRad_ = azC;
        elCentreRad_ = elC;
        azHalfRad_ = std::isfinite(task.azimuthHalfWidthRad) ? std::min(task.azimuthHalfWidthRad, azLimitRad_) : azLimitRad_;
        elHalfRad_ = std::isfinite(task.elevationHalfWidthRad) ? std::min(task.elevationHalfWidthRad, elLimitRad_) : elLimitRad_;
        rangeScaleM_ = std::isfinite(task.rangeScaleM) ? std::min(task.rangeScaleM, maxRangeM_) : std::min(160000.0, maxRangeM_);
        return Reason::None;
    }

    void stop(double) override {
        searching_ = false;
        finished_ = Reason::None;
        contacts_.clear();
        lastJamDegDb_ = 0.0;
    }

    Reason finished() const override { return finished_; }

    double percent() const override {
        if (!searching_ || !std::isfinite(durationS_) || durationS_ <= 0.0) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return std::clamp(100.0 * (lastS_ - startS_) / durationS_, 0.0, 100.0);
    }

    void step(double now, const sim::VehicleState& own, const Scene& scene, Rng& rng) override {
        if (!searching_) return;
        lastS_ = now;
        if (std::isfinite(durationS_) && now - startS_ + 1e-9 >= durationS_) {
            searching_ = false;
            finished_ = Reason::GoalReached;
            return;
        }
        if (now - lastScanS_ < 1.0 && !contacts_.empty()) return;
        lastScanS_ = now;
        contacts_.clear();

        // Evaluate in-band X-band RF noise jamming against weather radar receiver
        constexpr double kWxBwHz = 2.0e6;
        const double n0 = kBoltzmannT0 * std::pow(10.0, 5.0 / 10.0);
        double totalJ0 = 0.0;
        for (const Emission& e : scene.emissions()) {
            if (e.mode != EmitterMode::Jammer || !e.state || e.state == &own ||
                e.deceptionMode != EaDeceptionModeV1::Noise) continue;
            const double overlap = ea::spectralOverlap(e.frequencyHz, e.bandwidthHz, frequencyHz_, kWxBwHz);
            if (!(overlap > 0.0)) continue;
            double jLos[3];
            const double jr = geom::lineOfSight(own, e.state->positionEcef, jLos);
            if (!(jr > 1.0)) continue;
            double jBody[3], jAz = 0.0, jEl = 0.0;
            geom::toBody(own, jLos, jBody);
            geom::angles(jBody, jAz, jEl);
            const double rxGainDbi = geom::within(jAz, jEl, azCentreRad_, azHalfRad_, elCentreRad_, elHalfRad_)
                                         ? (peakGainDbi_ - 8.0)
                                         : -6.0;
            const double txGainDbi = e.gainTowardsDbi(own.positionEcef);
            const double lossDb = scene.pathLossDb(e.entity, frequencyHz_);
            totalJ0 += ea::receivedJammerDensityWHz(e.powerW, txGainDbi, rxGainDbi, lossDb, e.bandwidthHz, overlap);
        }
        lastJamDegDb_ = (totalJ0 > 0.0) ? (10.0 * std::log10(1.0 + totalJ0 / n0)) : 0.0;

        const auto& env = scene.environment();
        const double precip = std::clamp(std::isfinite(env.precipitation) ? env.precipitation : 0.0, 0.0, 1.0);
        // Map normalized environment precipitation (0..1) to baseline rain rate (mm/h)
        lastRainRateMmH_ = 50.0 * precip * precip;
        lastMaxDbz_ = -std::numeric_limits<double>::infinity();
        lastAttenDb_ = 0.0;
        if (!(lastRainRateMmH_ > 0.05)) return;

        const double rainTopM = (std::isfinite(env.cloudBaseM) ? env.cloudBaseM : 2000.0) + 5500.0;
        const double cellDepthM = std::clamp(rangeScaleM_ / 12.0, 5000.0, 10000.0);
        const int cellCount = std::min(12, std::max(1, static_cast<int>(rangeScaleM_ / cellDepthM)));

        // 2-D azimuth x slant-range sector scan (always centered on azCentreRad_, up to 7 azimuth rays)
        const int azRayCount = (azHalfRad_ > 8.0 * kDeg) ? 7 : ((azHalfRad_ > 2.0 * kDeg) ? 3 : 1);
        const double azStep = (azRayCount > 1) ? (2.0 * azHalfRad_ / static_cast<double>(azRayCount - 1)) : 0.0;
        const double cosEl = std::cos(elCentreRad_);
        const double sinEl = std::sin(elCentreRad_);

        for (int ia = 0; ia < azRayCount; ++ia) {
            const double rayAz = (azRayCount > 1)
                                     ? (azCentreRad_ - azHalfRad_ + static_cast<double>(ia) * azStep)
                                     : azCentreRad_;
            const double rayBody[3] = {
                cosEl * std::cos(rayAz),
                cosEl * std::sin(rayAz),
                -sinEl};
            double rayEcef[3];
            geom::toEcef(own, rayBody, rayEcef);

            // Independent per-ray two-way ITU-R P.838-3 rain attenuation integral (storm-core shadowing)
            double cumTwoWayAttenDb = 0.0;
            for (int k = 0; k < cellCount; ++k) {
                const double rangeM = (static_cast<double>(k) + 0.5) * cellDepthM;
                if (rangeM > rangeScaleM_) break;
                const double pt[3] = {
                    own.positionEcef[0] + rangeM * rayEcef[0],
                    own.positionEcef[1] + rangeM * rayEcef[1],
                    own.positionEcef[2] + rangeM * rayEcef[2]};
                double lat = 0.0, lon = 0.0, hGeomM = 0.0;
                ecefToGeodetic(pt, lat, lon, hGeomM);
                // Doviak & Zrnic (1993) Eq. 2.28: 4/3 effective-Earth refracted radar beam height
                const double beamHeightM = own.altitudeMslM + rangeM * sinEl +
                                           (rangeM * rangeM) / (2.0 * kEffEarthRadiusM);
                const double groundH = scene.surfaceHeightM(lat, lon);
                const double floorH = std::isfinite(groundH) ? groundH : 0.0;
                if (beamHeightM < floorH || beamHeightM > rainTopM) continue;

                const double localRainMmH = lastRainRateMmH_ * spatialConvectiveModulation(lat, lon);
                if (!(localRainMmH > 0.05)) continue;

                const double trueDbz = marshallPalmerDbz(localRainMmH);
                const double specAttenDbKm = rainSpecificAttenuationDbKm(localRainMmH, frequencyHz_);
                const double midAttenDb = cumTwoWayAttenDb + specAttenDbKm * (cellDepthM * 1e-3);
                cumTwoWayAttenDb += 2.0 * specAttenDbKm * (cellDepthM * 1e-3);

                const double measDbz = trueDbz - midAttenDb + 0.5 * rng.normal();
                // Probert-Jones (1962) R^2 beam-filled minimum detectable reflectivity raised by in-band X-band jamming
                const double mdsDbz = mdsDbzAt10Km_ + 20.0 * std::log10(std::max(rangeM, 1000.0) / 10000.0) + lastJamDegDb_;
                if (measDbz < mdsDbz) continue;

                Contact c;
                c.id = ++serial_;
                c.kind = ContactKind::Weather;
                c.warning = (measDbz >= 40.0); // Heavy convective storm cell (>= 40 dBZ)
                c.firstTimeS = now;
                c.timeS = now;
                c.azimuthRad = rayAz;
                c.elevationRad = elCentreRad_;
                c.bearingSigmaRad = std::max(0.5 * beamwidthRad_, 1.0 * kDeg);
                c.rangeM = rangeM;
                c.latitudeRad = lat;
                c.longitudeRad = lon;
                c.heightM = beamHeightM;
                c.strengthDb = measDbz;
                c.sizeM = cellDepthM;
                contacts_.push_back(c);

                lastMaxDbz_ = std::max(lastMaxDbz_, measDbz);
                lastAttenDb_ = std::max(lastAttenDb_, midAttenDb);
            }
        }
        // Order contacts strongest reflectivity first so cockpit/test consumers see primary convective cores first
        std::sort(contacts_.begin(), contacts_.end(), [](const Contact& a, const Contact& b) {
            return a.strengthDb > b.strengthDb;
        });
    }

    void idle(double now) override { stop(now); }

    void contacts(std::vector<Contact>& out) const override {
        for (const Contact& c : contacts_) out.push_back(c);
    }

    bool emission(Emission& out) const override {
        if (!searching_) return false;
        out.mode = EmitterMode::Search;
        out.frequencyHz = frequencyHz_;
        out.bandwidthHz = 2.0e6;
        out.powerW = powerW_;
        out.peakPowerW = peakPowerW_;
        out.peakGainDbi = peakGainDbi_;
        out.sidelobeDbi = -6.0;
        out.azimuthCentreRad = azCentreRad_;
        out.azimuthHalfWidthRad = azHalfRad_;
        out.elevationCentreRad = elCentreRad_;
        out.elevationHalfWidthRad = elHalfRad_;
        out.allRound = false;
        return true;
    }

    void report(std::vector<PayloadValue>& out) const override {
        out.push_back({"contacts", static_cast<double>(contacts_.size()), ""});
        out.push_back({"searching", searching_ ? 1.0 : 0.0, ""});
        out.push_back({"rain_rate", lastRainRateMmH_, "mm/h"});
        out.push_back({"max_dbz", std::isfinite(lastMaxDbz_) ? lastMaxDbz_ : std::numeric_limits<double>::quiet_NaN(), "dBZ"});
        out.push_back({"attenuation", lastAttenDb_, "dB"});
        out.push_back({"jam_degradation", lastJamDegDb_, "dB"});
        out.push_back({"frequency", frequencyHz_, "Hz"});
        out.push_back({"range_scale", rangeScaleM_, "m"});
    }

private:
    double frequencyHz_ = 9.375e9;
    double powerW_ = 120.0;
    double peakPowerW_ = 6000.0;
    double peakGainDbi_ = 30.0;
    double azLimitRad_ = 60.0 * kDeg;
    double elLimitRad_ = 15.0 * kDeg;
    double beamwidthRad_ = 3.5 * kDeg;
    double maxRangeM_ = 320000.0;
    double mdsDbzAt10Km_ = 5.0;

    bool searching_ = false;
    Reason finished_ = Reason::None;
    double startS_ = 0.0;
    double lastS_ = 0.0;
    double lastScanS_ = -std::numeric_limits<double>::infinity();
    double durationS_ = std::numeric_limits<double>::quiet_NaN();
    double azCentreRad_ = 0.0;
    double elCentreRad_ = 0.0;
    double azHalfRad_ = 60.0 * kDeg;
    double elHalfRad_ = 15.0 * kDeg;
    double rangeScaleM_ = 160000.0;
    double lastRainRateMmH_ = 0.0;
    double lastMaxDbz_ = -std::numeric_limits<double>::infinity();
    double lastAttenDb_ = 0.0;
    double lastJamDegDb_ = 0.0;
    std::vector<Contact> contacts_;
    std::uint64_t serial_ = 0;
};

} // namespace

double marshallPalmerDbz(double rainRateMmH) noexcept {
    if (!std::isfinite(rainRateMmH) || !(rainRateMmH > 0.0)) {
        return -std::numeric_limits<double>::infinity();
    }
    const double z = 200.0 * std::pow(rainRateMmH, 1.6);
    return 10.0 * std::log10(z);
}

double rainSpecificAttenuationDbKm(double rainRateMmH, double frequencyHz) noexcept {
    if (!std::isfinite(rainRateMmH) || !(rainRateMmH > 0.0)) return 0.0;
    // ITU-R P.838-3 horizontal polarization coefficients anchored at X-band (9.375 GHz: k=0.00887, alpha=1.264)
    const double fGhz = std::clamp(frequencyHz * 1e-9, 1.0, 35.0);
    const double scale = std::pow(fGhz / 9.375, 2.2);
    return 0.00887 * scale * std::pow(rainRateMmH, 1.264);
}

std::unique_ptr<Model> makeWeatherRadar(const Card& card, std::uint16_t count) {
    (void)count;
    return std::make_unique<WeatherRadar>(card);
}

} // namespace fsim::ms::weather
