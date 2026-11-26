#include "simulation/mission_systems/ground_map/GroundMapRadar.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fsim::ms::ground_map {

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

class GroundMapRadar final : public Model {
public:
    explicit GroundMapRadar(const Card& card) {
        frequencyHz_ = card.value("frequency_hz", 9.4e9);
        powerW_ = card.value("power_w", 600.0);
        peakPowerW_ = card.value("peak_power_w", 25000.0);
        peakGainDbi_ = card.value("peak_gain_dbi", 33.0);
        maxRangeM_ = card.value("range_max_m", 120000.0);
    }

    void step(double now, const sim::VehicleState& own, const Scene& scene, Rng&) override {
        active_ = true;
        if (now - lastScanS_ < 1.0 && !contacts_.empty()) return;
        lastScanS_ = now;
        contacts_.clear();
        minClearanceM_ = std::numeric_limits<double>::infinity();
        maxTerrainH_ = -std::numeric_limits<double>::infinity();

        const double fwdBody[3] = {1.0, 0.0, 0.0};
        double fwdEcef[3];
        geom::toEcef(own, fwdBody, fwdEcef);

        static constexpr double kSampleRangesM[4] = {5000.0, 15000.0, 30000.0, 60000.0};
        for (double r : kSampleRangesM) {
            if (r > maxRangeM_) continue;
            const double pt[3] = {
                own.positionEcef[0] + r * fwdEcef[0],
                own.positionEcef[1] + r * fwdEcef[1],
                own.positionEcef[2] + r * fwdEcef[2]};
            double lat = 0.0, lon = 0.0, hProj = 0.0;
            ecefToGeodetic(pt, lat, lon, hProj);
            const double groundH = scene.surfaceHeightM(lat, lon);
            const double terrH = std::isfinite(groundH) ? groundH : 0.0;
            const double clearance = own.altitudeMslM - terrH;
            minClearanceM_ = std::min(minClearanceM_, clearance);
            maxTerrainH_ = std::max(maxTerrainH_, terrH);

            Contact c;
            c.id = ++serial_;
            c.kind = ContactKind::Terrain;
            c.warning = (clearance < 300.0); // Terrain avoidance caution
            c.firstTimeS = now;
            c.timeS = now;
            c.azimuthRad = 0.0;
            c.elevationRad = std::atan2(terrH - own.altitudeMslM, r);
            c.bearingSigmaRad = 1.5 * kDeg;
            c.rangeM = r;
            c.latitudeRad = lat;
            c.longitudeRad = lon;
            c.heightM = terrH;
            c.strengthDb = -12.0; // Nominal terrain backscatter sigma0 (dB)
            c.sizeM = 5000.0;
            contacts_.push_back(c);
        }
    }

    void idle(double) override {
        active_ = false;
        contacts_.clear();
    }

    void contacts(std::vector<Contact>& out) const override {
        for (const Contact& c : contacts_) out.push_back(c);
    }

    bool emission(Emission& out) const override {
        if (!active_) return false;
        out.mode = EmitterMode::Search;
        out.frequencyHz = frequencyHz_;
        out.bandwidthHz = 5.0e6;
        out.powerW = powerW_;
        out.peakPowerW = peakPowerW_;
        out.peakGainDbi = peakGainDbi_;
        out.sidelobeDbi = -6.0;
        out.azimuthCentreRad = 0.0;
        out.azimuthHalfWidthRad = 45.0 * kDeg;
        out.elevationCentreRad = -10.0 * kDeg;
        out.elevationHalfWidthRad = 15.0 * kDeg;
        out.allRound = false;
        return true;
    }

    void report(std::vector<PayloadValue>& out) const override {
        out.push_back({"active", active_ ? 1.0 : 0.0, ""});
        out.push_back({"contacts", static_cast<double>(contacts_.size()), ""});
        out.push_back({"min_clearance", std::isfinite(minClearanceM_) ? minClearanceM_ : std::numeric_limits<double>::quiet_NaN(), "m"});
        out.push_back({"max_terrain_h", std::isfinite(maxTerrainH_) ? maxTerrainH_ : std::numeric_limits<double>::quiet_NaN(), "m"});
        out.push_back({"frequency", frequencyHz_, "Hz"});
    }

private:
    double frequencyHz_ = 9.4e9;
    double powerW_ = 600.0;
    double peakPowerW_ = 25000.0;
    double peakGainDbi_ = 33.0;
    double maxRangeM_ = 120000.0;
    bool active_ = false;
    double lastScanS_ = -std::numeric_limits<double>::infinity();
    double minClearanceM_ = std::numeric_limits<double>::infinity();
    double maxTerrainH_ = -std::numeric_limits<double>::infinity();
    std::vector<Contact> contacts_;
    std::uint64_t serial_ = 0;
};

} // namespace

std::unique_ptr<Model> makeGroundMapRadar(const Card& card, std::uint16_t count) {
    (void)count;
    return std::make_unique<GroundMapRadar>(card);
}

} // namespace fsim::ms::ground_map
