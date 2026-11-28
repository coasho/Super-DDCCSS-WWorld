#include "simulation/mission_systems/smti/SmtiRadar.h"
#include "simulation/mission_systems/ea/Jammer.h"
#include "simulation/mission_systems/radar/Detection.h"
#include "simulation/mission_systems/radar/Radar.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fsim::ms::smti {

namespace {

constexpr double kPi = geom::kPi;
constexpr double kDeg = kPi / 180.0;
constexpr double kLight = 299792458.0;
constexpr double kBoltzmannT0 = 1.380649e-23 * 290.0;

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

class SmtiRadar final : public Model {
public:
    explicit SmtiRadar(const Card& card) {
        frequencyHz_ = card.value("frequency_hz", 9.5e9);
        clutterSpreadHz_ = card.value("clutter_spread_hz", 160.0); // ~2.5 m/s MDV at X-band
        powerW_ = card.value("power_w", 1500.0);
        peakPowerW_ = card.value("peak_power_w", 15000.0);
        peakGainDbi_ = card.value("peak_gain_dbi", 36.0);
        beamwidthRad_ = card.value("beamwidth_deg", 1.8) * kDeg;
        rangeResM_ = card.value("range_resolution_m", 15.0);
        dwellS_ = card.value("dwell_s", 0.05);
        noiseFigureDb_ = card.value("noise_figure_db", 4.5);
        systemLossDb_ = card.value("system_loss_db", 6.0);
        // STAP / DPCA endo-clutter improvement factor (Richards / Klemm, typical airborne GMTI 35-45 dB)
        clutterImprovementDb_ = card.value("clutter_improvement_db", 42.0);
        maxRangeM_ = card.value("range_max_m", 180000.0);
        azLimitRad_ = card.value("azimuth_limit_deg", 60.0) * kDeg;
        mdvMs_ = minimumDetectableVelocityMs(frequencyHz_, clutterSpreadHz_);
    }

    Reason begin(const PayloadTask& task, double now) override {
        if (task.kind != TaskKind::Search) return Reason::InvalidParameter;
        if (!std::isfinite(task.azimuthCentreRad) || !std::isfinite(task.elevationCentreRad)) return Reason::InvalidParameter;
        if (std::isfinite(task.rangeScaleM) && !(task.rangeScaleM > 0.0)) return Reason::InvalidParameter;
        if (std::isfinite(task.durationS) && !(task.durationS > 0.0)) return Reason::InvalidParameter;

        searching_ = true;
        finished_ = Reason::None;
        startS_ = now;
        lastS_ = now;
        durationS_ = std::isfinite(task.durationS) ? task.durationS : std::numeric_limits<double>::quiet_NaN();
        azCentreRad_ = std::remainder(task.azimuthCentreRad, 2.0 * kPi);
        elCentreRad_ = std::clamp(task.elevationCentreRad, -0.5 * kPi, 0.5 * kPi);
        azHalfRad_ = std::isfinite(task.azimuthHalfWidthRad) ? std::min(task.azimuthHalfWidthRad, azLimitRad_) : azLimitRad_;
        elHalfRad_ = std::isfinite(task.elevationHalfWidthRad) ? std::min(task.elevationHalfWidthRad, 0.5 * kPi) : 30.0 * kDeg;
        rangeScaleM_ = std::isfinite(task.rangeScaleM) ? std::min(task.rangeScaleM, maxRangeM_) : maxRangeM_;
        return Reason::None;
    }

    void stop(double) override {
        searching_ = false;
        finished_ = Reason::None;
        contacts_.clear();
        lastJamDegDb_ = 0.0;
    }

    Reason finished() const override { return finished_; }

    void step(double now, const sim::VehicleState& own, const Scene& scene, Rng& rng) override {
        if (!searching_) return;
        lastS_ = now;
        if (std::isfinite(durationS_) && now - startS_ + 1e-9 >= durationS_) {
            searching_ = false;
            finished_ = Reason::GoalReached;
            return;
        }
        contacts_.clear();

        // Evaluate in-band RF noise jamming density J0 against SMTI receiver
        const double lambda = kLight / std::max(frequencyHz_, 1.0);
        const double rxBwHz = std::max(1.0 / std::max(dwellS_, 1e-4), 0.5 * kLight / std::max(rangeResM_, 1.0));
        const double n0 = kBoltzmannT0 * std::pow(10.0, noiseFigureDb_ / 10.0);
        double totalJ0 = 0.0;
        for (const Emission& e : scene.emissions()) {
            if (e.mode != EmitterMode::Jammer || !e.state || e.state == &own ||
                e.deceptionMode != EaDeceptionModeV1::Noise) continue;
            const double overlap = ea::spectralOverlap(e.frequencyHz, e.bandwidthHz, frequencyHz_, rxBwHz);
            if (!(overlap > 0.0)) continue;
            double jLos[3];
            const double jr = geom::lineOfSight(own, e.state->positionEcef, jLos);
            if (!(jr > 1.0)) continue;
            double jBody[3], jAz = 0.0, jEl = 0.0;
            geom::toBody(own, jLos, jBody);
            geom::angles(jBody, jAz, jEl);
            const double rxGainDbi = geom::within(jAz, jEl, azCentreRad_, azHalfRad_, elCentreRad_, elHalfRad_)
                                         ? (peakGainDbi_ - 8.0)
                                         : -8.0;
            const double txGainDbi = e.gainTowardsDbi(own.positionEcef);
            const double lossDb = scene.pathLossDb(e.entity, frequencyHz_);
            totalJ0 += ea::receivedJammerDensityWHz(e.powerW, txGainDbi, rxGainDbi, lossDb, e.bandwidthHz, overlap);
        }
        lastJamDegDb_ = (totalJ0 > 0.0) ? (10.0 * std::log10(1.0 + totalJ0 / n0)) : 0.0;

        const double gLin = std::pow(10.0, peakGainDbi_ / 10.0);
        const double sysLossLin = std::pow(10.0, systemLossDb_ / 10.0);
        const double mtiCancelLin = std::pow(10.0, clutterImprovementDb_ / 10.0);
        const double freqDb = 5.0 * std::log10(std::max(frequencyHz_, 1.0) / 10.0e9);
        const double gammaLand = std::pow(10.0, (radar::Radar::kGammaLandDb + freqDb) / 10.0);
        const double gammaSea = std::pow(10.0, (radar::Radar::kGammaSeaDb + freqDb) / 10.0);
        constexpr double kPfa = 1.0e-6;

        std::uint32_t ids[512];
        const sim::VehicleState* targets[512];
        const std::size_t n = scene.targets(ids, targets, 512);
        double ownVel[3];
        geom::velocityEcef(own, ownVel);
        for (std::size_t k = 0; k < n; ++k) {
            if (!targets[k]) continue;
            const sim::VehicleState& st = *targets[k];
            // SMTI tracks surface / near-surface movers only (onGround or < 150 m AGL)
            if (!st.onGround && st.altitudeAglM > 150.0) continue;
            double los[3];
            const double rangeM = geom::lineOfSight(own, st.positionEcef, los);
            if (!(rangeM > 1.0 && rangeM <= rangeScaleM_) || scene.masked(ids[k], rangeM)) continue;
            double body[3], az = 0.0, el = 0.0;
            geom::toBody(own, los, body);
            geom::angles(body, az, el);
            if (!geom::within(az, el, azCentreRad_, azHalfRad_, elCentreRad_, elHalfRad_)) continue;

            double tVel[3];
            geom::velocityEcef(st, tVel);
            // Radial speed relative to stationary ground clutter along LOS
            const double groundRadial = std::fabs(geom::dot3(tVel, los));
            if (groundRadial < mdvMs_) continue;

            // Physical RCS query + Swerling fluctuation model (Meyer & Mayer 1973 / Richards Ch. 6)
            double fluct = 2.0; // Swerling 1 default if no RCS table
            double sigmaM2 = scene.sigma(ids[k], frequencyHz_, 0.0, fluct);
            if (!std::isfinite(sigmaM2) || !(sigmaM2 > 0.0)) sigmaM2 = 10.0; // Generic vehicle fallback

            const double lossOneWayDb = scene.pathLossDb(ids[k], frequencyHz_);
            const double lossOneWayLin = std::pow(10.0, lossOneWayDb / 10.0);
            const double sigEnergyJ = (powerW_ * gLin * gLin * (4.0 * kPi * sigmaM2 / (lambda * lambda)) * dwellS_) /
                                      (lossOneWayLin * lossOneWayLin * sysLossLin);

            // Residual mainlobe ground clutter after STAP/DPCA cancellation (Nathanson/Long constant-gamma)
            const double groundH = scene.surfaceHeightM(st.latitudeRad, st.longitudeRad);
            const bool overSea = std::isfinite(groundH) && groundH <= 0.0;
            const double sinPsi = std::clamp(std::max(10.0, own.altitudeMslM - st.altitudeMslM) / rangeM, 0.005, 0.995);
            const double cosPsi = std::sqrt(std::max(1e-4, 1.0 - sinPsi * sinPsi));
            const double clutterSigmaM2 = (overSea ? gammaSea : gammaLand) * sinPsi *
                                          (rangeM * beamwidthRad_ * rangeResM_ / cosPsi);
            const double clutterEnergyJ = (powerW_ * gLin * gLin * (4.0 * kPi * clutterSigmaM2 / (lambda * lambda)) * dwellS_) /
                                          (lossOneWayLin * lossOneWayLin * sysLossLin * mtiCancelLin);

            const double totalNoiseJ = n0 + totalJ0 + clutterEnergyJ;
            const double sinrLin = sigEnergyJ / std::max(totalNoiseJ, 1e-35);
            const double pd = radar::probabilityOfDetection(sinrLin, kPfa, radar::fluctuationOf(fluct));
            if (rng.uniform() >= pd) continue;

            // Cramer-Rao SNR-scaled measurements (Richards / Skolnik); never export raw truth lat/lon/alt
            const double root = std::sqrt(2.0 * std::max(sinrLin, 1.0));
            const double sigmaAngle = std::hypot(beamwidthRad_ / (1.6 * root), 0.05 * beamwidthRad_);
            const double sigmaRange = std::hypot(rangeResM_ / root, 0.10 * rangeResM_);
            const double dopplerRes = lambda / (2.0 * std::max(dwellS_, 1e-4));
            const double sigmaRate = std::hypot(dopplerRes / root, 0.15);

            const double measAz = az + sigmaAngle * rng.normal();
            const double measEl = std::clamp(el + sigmaAngle * rng.normal(), -0.49 * kPi, 0.49 * kPi);
            const double measRange = std::max(10.0, rangeM + sigmaRange * rng.normal());
            const double trueRate = geom::dot3(tVel, los) - geom::dot3(ownVel, los);
            const double measRate = trueRate + sigmaRate * rng.normal();

            // Back-project noisy measurement (az, el, range) into estimated ECEF -> geodetic coordinates
            const double ce = std::cos(measEl);
            const double measBody[3] = {
                measRange * ce * std::cos(measAz),
                measRange * ce * std::sin(measAz),
                -measRange * std::sin(measEl)};
            double measRelEcef[3];
            geom::toEcef(own, measBody, measRelEcef);
            const double measEcef[3] = {
                own.positionEcef[0] + measRelEcef[0],
                own.positionEcef[1] + measRelEcef[1],
                own.positionEcef[2] + measRelEcef[2]};
            double estLat = 0.0, estLon = 0.0, estH = 0.0;
            ecefToGeodetic(measEcef, estLat, estLon, estH);

            Contact c;
            c.id = ++serial_;
            c.kind = ContactKind::Surface;
            c.firstTimeS = now;
            c.timeS = now;
            c.azimuthRad = measAz;
            c.elevationRad = measEl;
            c.bearingSigmaRad = sigmaAngle;
            c.rangeM = measRange;
            c.rangeRateMs = measRate;
            c.latitudeRad = estLat;
            c.longitudeRad = estLon;
            c.heightM = estH;
            c.strengthDb = 10.0 * std::log10(std::max(sinrLin, 1e-6));
            c.truth = ids[k];
            contacts_.push_back(c);
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
        out.bandwidthHz = std::max(1.0e6, 0.5 * kLight / std::max(rangeResM_, 1.0));
        out.powerW = powerW_;
        out.peakPowerW = peakPowerW_;
        out.peakGainDbi = peakGainDbi_;
        out.sidelobeDbi = -8.0;
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
        out.push_back({"mdv", mdvMs_, "m/s"});
        out.push_back({"jam_degradation", lastJamDegDb_, "dB"});
        out.push_back({"frequency", frequencyHz_, "Hz"});
    }

private:
    double frequencyHz_ = 9.5e9;
    double clutterSpreadHz_ = 160.0;
    double mdvMs_ = 2.5;
    double powerW_ = 1500.0;
    double peakPowerW_ = 15000.0;
    double peakGainDbi_ = 36.0;
    double beamwidthRad_ = 1.8 * kDeg;
    double rangeResM_ = 15.0;
    double dwellS_ = 0.05;
    double noiseFigureDb_ = 4.5;
    double systemLossDb_ = 6.0;
    double clutterImprovementDb_ = 42.0;
    double maxRangeM_ = 180000.0;
    double azLimitRad_ = 60.0 * kDeg;
    bool searching_ = false;
    Reason finished_ = Reason::None;
    double startS_ = 0.0;
    double lastS_ = 0.0;
    double durationS_ = std::numeric_limits<double>::quiet_NaN();
    double azCentreRad_ = 0.0;
    double elCentreRad_ = 0.0;
    double azHalfRad_ = 60.0 * kDeg;
    double elHalfRad_ = 30.0 * kDeg;
    double rangeScaleM_ = 180000.0;
    double lastJamDegDb_ = 0.0;
    std::vector<Contact> contacts_;
    std::uint64_t serial_ = 0;
};

} // namespace

double minimumDetectableVelocityMs(double frequencyHz, double clutterSpreadHz) noexcept {
    if (!(frequencyHz > 0.0) || !(clutterSpreadHz >= 0.0)) return 0.0;
    return 0.5 * (kLight / frequencyHz) * clutterSpreadHz;
}

std::unique_ptr<Model> makeSmtiRadar(const Card& card, std::uint16_t count) {
    (void)count;
    return std::make_unique<SmtiRadar>(card);
}

} // namespace fsim::ms::smti
