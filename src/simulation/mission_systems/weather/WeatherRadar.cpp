#include "simulation/mission_systems/weather/WeatherRadar.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fsim::ms::weather {

namespace {

constexpr double kPi = geom::kPi;
constexpr double kDeg = kPi / 180.0;

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

class WeatherRadar final : public Model {
public:
    explicit WeatherRadar(const Card& card) {
        frequencyHz_ = card.value("frequency_hz", 9.375e9);
        powerW_ = card.value("power_w", 120.0);
        peakPowerW_ = card.value("peak_power_w", 6000.0);
        peakGainDbi_ = card.value("peak_gain_dbi", 30.0);
        azLimitRad_ = card.value("azimuth_limit_deg", 60.0) * kDeg;
        elLimitRad_ = card.value("elevation_limit_deg", 15.0) * kDeg;
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

        const auto& env = scene.environment();
        const double precip = std::clamp(std::isfinite(env.precipitation) ? env.precipitation : 0.0, 0.0, 1.0);
        // Map normalized environment precipitation (0..1) to physical convective/stratiform rain rate (mm/h)
        lastRainRateMmH_ = 50.0 * precip * precip;
        lastMaxDbz_ = -std::numeric_limits<double>::infinity();
        lastAttenDb_ = 0.0;
        if (!(lastRainRateMmH_ > 0.05)) return;

        const double trueDbz = marshallPalmerDbz(lastRainRateMmH_);
        const double specAttenDbKm = rainSpecificAttenuationDbKm(lastRainRateMmH_, frequencyHz_);
        const double rainTopM = (std::isfinite(env.cloudBaseM) ? env.cloudBaseM : 2000.0) + 4000.0;

        // Sample radial weather cells along the scan centreline within rangeScaleM_
        const double cellDepthM = std::clamp(rangeScaleM_ / 12.0, 4000.0, 10000.0);
        const int cellCount = std::min(12, std::max(1, static_cast<int>(rangeScaleM_ / cellDepthM)));
        const double rayBody[3] = {
            std::cos(elCentreRad_) * std::cos(azCentreRad_),
            std::cos(elCentreRad_) * std::sin(azCentreRad_),
            -std::sin(elCentreRad_)};
        double rayEcef[3];
        geom::toEcef(own, rayBody, rayEcef);

        double cumTwoWayAttenDb = 0.0;
        for (int k = 0; k < cellCount; ++k) {
            const double rangeM = (static_cast<double>(k) + 0.5) * cellDepthM;
            if (rangeM > rangeScaleM_) break;
            const double pt[3] = {
                own.positionEcef[0] + rangeM * rayEcef[0],
                own.positionEcef[1] + rangeM * rayEcef[1],
                own.positionEcef[2] + rangeM * rayEcef[2]};
            double lat = 0.0, lon = 0.0, hM = 0.0;
            ecefToGeodetic(pt, lat, lon, hM);
            if (hM < 0.0 || hM > rainTopM) continue;

            // Two-way X-band rain attenuation to cell midpoint, then accumulate remaining half-cell
            const double midAttenDb = cumTwoWayAttenDb + specAttenDbKm * (cellDepthM * 1e-3);
            cumTwoWayAttenDb += 2.0 * specAttenDbKm * (cellDepthM * 1e-3);
            const double measDbz = trueDbz - midAttenDb + 0.5 * rng.normal();
            // Probert-Jones R^2 beam-filled minimum detectable reflectivity
            const double mdsDbz = mdsDbzAt10Km_ + 20.0 * std::log10(std::max(rangeM, 1000.0) / 10000.0);
            if (measDbz < mdsDbz) continue;

            Contact c;
            c.id = ++serial_;
            c.kind = ContactKind::Weather;
            c.warning = (measDbz >= 40.0); // Heavy convective storm cell (>= 40 dBZ)
            c.firstTimeS = now;
            c.timeS = now;
            c.azimuthRad = azCentreRad_;
            c.elevationRad = elCentreRad_;
            c.bearingSigmaRad = 2.0 * kDeg;
            c.rangeM = rangeM;
            c.latitudeRad = lat;
            c.longitudeRad = lon;
            c.heightM = hM;
            c.strengthDb = measDbz;
            c.sizeM = cellDepthM;
            contacts_.push_back(c);

            lastMaxDbz_ = std::max(lastMaxDbz_, measDbz);
            lastAttenDb_ = std::max(lastAttenDb_, midAttenDb);
        }
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
