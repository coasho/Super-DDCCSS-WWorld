#include "simulation/mission_systems/ground_map/GroundMapRadar.h"
#include "simulation/mission_systems/ea/Jammer.h"
#include "simulation/mission_systems/radar/Radar.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fsim::ms::ground_map {

namespace {

constexpr double kPi = geom::kPi;
constexpr double kDeg = kPi / 180.0;
constexpr double kLight = 299792458.0;
constexpr double kBoltzmannT0 = 1.380649e-23 * 290.0;
constexpr double kEffEarthRadiusM = (4.0 / 3.0) * 6371000.0; // Standard 4/3-Earth RF refractivity (Doviak & Zrnic / Barton)

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
        azLimitRad_ = card.value("azimuth_limit_deg", 45.0) * kDeg;
        beamwidthRad_ = card.value("beamwidth_deg", 2.5) * kDeg;
        rangeResM_ = card.value("range_resolution_m", 30.0);
        noiseFigureDb_ = card.value("noise_figure_db", 5.0);
        systemLossDb_ = card.value("system_loss_db", 6.0);
        dwellS_ = card.value("dwell_s", 0.02);
        maxRangeM_ = card.value("range_max_m", 120000.0);
    }

    void step(double now, const sim::VehicleState& own, const Scene& scene, Rng& rng) override {
        active_ = true;
        if (now - lastScanS_ < 1.0 && !contacts_.empty()) return;
        lastScanS_ = now;
        contacts_.clear();
        minClearanceM_ = std::numeric_limits<double>::infinity();
        maxTerrainH_ = -std::numeric_limits<double>::infinity();
        shadowedCells_ = 0;
        lastJamDegDb_ = 0.0;

        // Evaluate in-band RF noise jamming against real-beam ground-map receiver
        const double rxBwHz = std::max(1.0e6, 0.5 * kLight / std::max(rangeResM_, 1.0));
        const double n0 = kBoltzmannT0 * std::pow(10.0, noiseFigureDb_ / 10.0);
        double totalJ0 = 0.0;
        for (const Emission& e : scene.emissions()) {
            if (e.mode != EmitterMode::Jammer || !e.state || e.state == &own ||
                e.deceptionMode != EaDeceptionModeV1::Noise) continue;
            const double overlap = ea::spectralOverlap(e.frequencyHz, e.bandwidthHz, frequencyHz_, rxBwHz);
            if (!(overlap > 0.0)) continue;
            double los[3];
            const double r = geom::lineOfSight(own, e.state->positionEcef, los);
            if (!(r > 1.0)) continue;
            double body[3], jAz = 0.0, jEl = 0.0;
            geom::toBody(own, los, body);
            geom::angles(body, jAz, jEl);
            // Main-sector average coupling vs sidelobe coupling for scanning real-beam dish
            const double rxGainDbi = (std::fabs(jAz) <= azLimitRad_ && jEl >= -40.0 * kDeg && jEl <= 15.0 * kDeg)
                                         ? (peakGainDbi_ - 10.0)
                                         : -6.0;
            const double txGainDbi = e.gainTowardsDbi(own.positionEcef);
            const double lossDb = scene.pathLossDb(e.entity, frequencyHz_);
            totalJ0 += ea::receivedJammerDensityWHz(e.powerW, txGainDbi, rxGainDbi, lossDb, e.bandwidthHz, overlap);
        }
        lastJamDegDb_ = (totalJ0 > 0.0) ? (10.0 * std::log10(1.0 + totalJ0 / n0)) : 0.0;

        // Constant-gamma land/sea surface backscatter model (Nathanson 1999; Long 2001; consistent with Radar.h)
        const double lambda = kLight / std::max(frequencyHz_, 1.0);
        const double freqDb = 5.0 * std::log10(std::max(frequencyHz_, 1.0) / 10.0e9);
        const double gammaLand = std::pow(10.0, (radar::Radar::kGammaLandDb + freqDb) / 10.0);
        const double gammaSea = std::pow(10.0, (radar::Radar::kGammaSeaDb + freqDb) / 10.0);

        const double gLin = std::pow(10.0, peakGainDbi_ / 10.0);
        const double lossSysLin = std::pow(10.0, systemLossDb_ / 10.0);
        const double effNoiseEnergyJ = (n0 + totalJ0); // J (= W/Hz), matched against coherent dwell energy P_r * T_dwell
        const double snrMinLin = 10.0;                 // 10 dB real-beam mapping display detection threshold

        const double belowH = scene.surfaceHeightM(own.latitudeRad, own.longitudeRad);
        const double aglM = std::max(30.0, std::isfinite(belowH) ? (own.altitudeMslM - belowH) : own.altitudeAglM);
        const double horizonM = std::sqrt(2.0 * kEffEarthRadiusM * aglM);
        const double rMax = std::min(maxRangeM_, std::max(5000.0, horizonM));
        const double rMin = std::clamp(1.2 * aglM, 2500.0, 12000.0);
        if (!(rMax > rMin)) return;

        // Scan 7 azimuth rays across [-azLimitRad_, +azLimitRad_] and 7 range bins per ray with ridge LOS shadowing.
        constexpr int kAzRays = 7;
        constexpr int kRangeBins = 7;
        const double ratio = std::pow(rMax / rMin, 1.0 / static_cast<double>(kRangeBins - 1));
        const double azStep = (kAzRays > 1) ? (2.0 * azLimitRad_ / static_cast<double>(kAzRays - 1)) : 0.0;

        for (int ia = 0; ia < kAzRays; ++ia) {
            const double az = -azLimitRad_ + static_cast<double>(ia) * azStep;
            const double rayBody[3] = {std::cos(az), std::sin(az), 0.0};
            double rayEcef[3];
            geom::toEcef(own, rayBody, rayEcef);

            // Track highest terrain grazing elevation seen so far along this azimuth ray for true ridge shadowing.
            double maxElevRad = -0.5 * kPi;
            for (int ir = 0; ir < kRangeBins; ++ir) {
                const double rGround = rMin * std::pow(ratio, static_cast<double>(ir));
                if (rGround > rMax) break;
                const double pt[3] = {
                    own.positionEcef[0] + rGround * rayEcef[0],
                    own.positionEcef[1] + rGround * rayEcef[1],
                    own.positionEcef[2] + rGround * rayEcef[2]};
                double lat = 0.0, lon = 0.0, hProj = 0.0;
                ecefToGeodetic(pt, lat, lon, hProj);
                const double rawGroundH = scene.surfaceHeightM(lat, lon);
                const bool isSea = std::isfinite(rawGroundH) && rawGroundH <= 0.0;
                const double terrH = std::isfinite(rawGroundH) ? rawGroundH : 0.0;

                // 4/3-Earth curvature drop along ray: delta_h_curv = r^2 / (2 * R_eff)
                const double curvDropM = (rGround * rGround) / (2.0 * kEffEarthRadiusM);
                const double dh = (terrH - curvDropM) - own.altitudeMslM;
                const double slantRangeM = std::hypot(rGround, dh);
                const double rayElevRad = std::atan2(dh, rGround);

                // Ridge occlusion: terrain falling below a closer ridge's sightline lies in radar shadow
                if (rayElevRad < maxElevRad - 1e-4) {
                    ++shadowedCells_;
                    continue;
                }
                maxElevRad = rayElevRad;

                const double clearance = own.altitudeMslM - terrH;
                minClearanceM_ = std::min(minClearanceM_, clearance);
                maxTerrainH_ = std::max(maxTerrainH_, terrH);

                // Grazing angle psi at surface patch (curved 4/3-Earth)
                const double sinPsi = std::clamp(
                    (own.altitudeMslM - terrH - curvDropM) / std::max(slantRangeM, 1.0), 0.005, 0.995);
                const double cosPsi = std::sqrt(std::max(1e-4, 1.0 - sinPsi * sinPsi));
                const double gamma = isSea ? gammaSea : gammaLand;
                const double sigma0 = gamma * sinPsi;

                // Monostatic area-clutter radar equation (Skolnik / Barton):
                // A_cell = R * theta_az * dR / cos(psi); sigma_cell = sigma0 * A_cell
                const double cellAreaM2 = slantRangeM * beamwidthRad_ * rangeResM_ / cosPsi;
                const double sigmaCellM2 = sigma0 * cellAreaM2;
                const double prW = (powerW_ * gLin * gLin * lambda * lambda * sigmaCellM2) /
                                   (std::pow(4.0 * kPi, 3) * std::pow(slantRangeM, 4) * lossSysLin);
                const double snrLin = (prW * dwellS_) / std::max(effNoiseEnergyJ, 1e-35);
                if (snrLin < snrMinLin) continue;

                // SNR-scaled real-beam range and bearing measurement uncertainty
                const double root = std::sqrt(2.0 * snrLin);
                const double sigmaAz = std::hypot(beamwidthRad_ / (1.6 * root), 0.15 * beamwidthRad_);
                const double sigmaR = std::hypot(rangeResM_ / root, 0.25 * rangeResM_);
                const double measAz = az + sigmaAz * rng.normal();
                const double measEl = rayElevRad + sigmaAz * rng.normal();
                const double measR = std::max(100.0, slantRangeM + sigmaR * rng.normal());
                const double sigma0Db = 10.0 * std::log10(std::max(sigma0, 1e-12)) + 0.8 * rng.normal();

                Contact c;
                c.id = ++serial_;
                c.kind = ContactKind::Terrain;
                c.warning = (clearance < 300.0); // Terrain avoidance caution
                c.firstTimeS = now;
                c.timeS = now;
                c.azimuthRad = measAz;
                c.elevationRad = measEl;
                c.bearingSigmaRad = sigmaAz;
                c.rangeM = measR;
                c.latitudeRad = lat;
                c.longitudeRad = lon;
                c.heightM = terrH;
                c.strengthDb = sigma0Db;
                c.sizeM = std::max(rangeResM_, slantRangeM * beamwidthRad_);
                contacts_.push_back(c);
            }
        }
    }

    void idle(double) override {
        active_ = false;
        contacts_.clear();
        shadowedCells_ = 0;
        lastJamDegDb_ = 0.0;
    }

    void contacts(std::vector<Contact>& out) const override {
        for (const Contact& c : contacts_) out.push_back(c);
    }

    bool emission(Emission& out) const override {
        if (!active_) return false;
        out.mode = EmitterMode::Search;
        out.frequencyHz = frequencyHz_;
        out.bandwidthHz = std::max(1.0e6, 0.5 * kLight / std::max(rangeResM_, 1.0));
        out.powerW = powerW_;
        out.peakPowerW = peakPowerW_;
        out.peakGainDbi = peakGainDbi_;
        out.sidelobeDbi = -6.0;
        out.azimuthCentreRad = 0.0;
        out.azimuthHalfWidthRad = azLimitRad_;
        out.elevationCentreRad = -10.0 * kDeg;
        out.elevationHalfWidthRad = 15.0 * kDeg;
        out.allRound = false;
        return true;
    }

    void report(std::vector<PayloadValue>& out) const override {
        out.push_back({"active", active_ ? 1.0 : 0.0, ""});
        out.push_back({"contacts", static_cast<double>(contacts_.size()), ""});
        out.push_back({"shadowed_cells", static_cast<double>(shadowedCells_), ""});
        out.push_back({"jam_degradation", lastJamDegDb_, "dB"});
        out.push_back({"min_clearance", std::isfinite(minClearanceM_) ? minClearanceM_ : std::numeric_limits<double>::quiet_NaN(), "m"});
        out.push_back({"max_terrain_h", std::isfinite(maxTerrainH_) ? maxTerrainH_ : std::numeric_limits<double>::quiet_NaN(), "m"});
        out.push_back({"frequency", frequencyHz_, "Hz"});
    }

private:
    double frequencyHz_ = 9.4e9;
    double powerW_ = 600.0;
    double peakPowerW_ = 25000.0;
    double peakGainDbi_ = 33.0;
    double azLimitRad_ = 45.0 * kDeg;
    double beamwidthRad_ = 2.5 * kDeg;
    double rangeResM_ = 30.0;
    double noiseFigureDb_ = 5.0;
    double systemLossDb_ = 6.0;
    double dwellS_ = 0.02;
    double maxRangeM_ = 120000.0;

    bool active_ = false;
    double lastScanS_ = -std::numeric_limits<double>::infinity();
    double minClearanceM_ = std::numeric_limits<double>::infinity();
    double maxTerrainH_ = -std::numeric_limits<double>::infinity();
    std::size_t shadowedCells_ = 0;
    double lastJamDegDb_ = 0.0;
    std::vector<Contact> contacts_;
    std::uint64_t serial_ = 0;
};

} // namespace

std::unique_ptr<Model> makeGroundMapRadar(const Card& card, std::uint16_t count) {
    (void)count;
    return std::make_unique<GroundMapRadar>(card);
}

} // namespace fsim::ms::ground_map
