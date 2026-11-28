#include "simulation/mission_systems/sar/SarRadar.h"
#include "simulation/mission_systems/ea/Jammer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fsim::ms::sar {

namespace {

constexpr double kPi = geom::kPi;
constexpr double kDeg = kPi / 180.0;
constexpr double kLight = 299792458.0;
constexpr double kBoltzmannT0 = 1.380649e-23 * 290.0;
constexpr double kEarthA = 6378137.0;
constexpr double kEarthE2 = 0.00669437999014;

void geodeticToEcef(double latRad, double lonRad, double heightM, double out[3]) noexcept {
    const double sl = std::sin(latRad), cl = std::cos(latRad);
    const double so = std::sin(lonRad), co = std::cos(lonRad);
    const double n = kEarthA / std::sqrt(1.0 - kEarthE2 * sl * sl);
    out[0] = (n + heightM) * cl * co;
    out[1] = (n + heightM) * cl * so;
    out[2] = (n * (1.0 - kEarthE2) + heightM) * sl;
}

class SarRadar final : public Model {
public:
    explicit SarRadar(const Card& card) {
        // Generic X-band SAR fallback when a synthetic card omits parameters;
        // maintained sar cards declare their own parameters in assets/payloads/*.json.
        frequencyHz_ = card.value("frequency_hz", 9.6e9);
        bandwidthHz_ = card.value("bandwidth_hz", 300.0e6);
        avgPowerW_ = card.value("power_w", 1200.0);
        peakPowerW_ = card.value("peak_power_w", 12000.0);
        antennaGainDbi_ = card.value("peak_gain_dbi", 36.0);
        noiseFigureDb_ = card.value("noise_figure_db", 4.0);
        systemLossDb_ = card.value("system_loss_db", 5.0);
        finestResM_ = card.value("finest_resolution_m", 0.3);
        minRangeM_ = card.value("range_min_m", 15000.0);
        maxRangeM_ = card.value("range_max_m", 200000.0);
    }

    Reason begin(const PayloadTask& task, double now) override {
        if (task.kind != TaskKind::Image) return Reason::InvalidParameter;
        if (!std::isfinite(task.latitudeRad) || !std::isfinite(task.longitudeRad) ||
            std::fabs(task.latitudeRad) > 0.5 * kPi || std::fabs(task.longitudeRad) > kPi) {
            return Reason::InvalidParameter;
        }
        const double res = std::isfinite(task.resolutionM) ? task.resolutionM : 1.0;
        if (!(res >= finestResM_ - 1e-9) || res > 50.0) return Reason::InvalidParameter;
        if (std::isfinite(task.durationS) && !(task.durationS > 0.0)) return Reason::InvalidParameter;

        collecting_ = true;
        geometryChecked_ = false;
        finished_ = Reason::None;
        startS_ = now;
        lastS_ = now;
        targetLatRad_ = task.latitudeRad;
        targetLonRad_ = task.longitudeRad;
        resolutionM_ = res;
        requiredIntS_ = 1.0;
        accumIntS_ = 0.0;
        lastNeszDb_ = std::numeric_limits<double>::quiet_NaN();
        lastJamDegDb_ = 0.0;
        return Reason::None;
    }

    void stop(double) override {
        collecting_ = false;
        geometryChecked_ = false;
        finished_ = Reason::None;
    }

    Reason finished() const override { return finished_; }

    double percent() const override {
        if (!collecting_ || !(requiredIntS_ > 0.0)) return std::numeric_limits<double>::quiet_NaN();
        return std::clamp(100.0 * accumIntS_ / requiredIntS_, 0.0, 100.0);
    }

    void step(double now, const sim::VehicleState& own, const Scene& scene, Rng&) override {
        if (!collecting_) return;
        const double dt = std::max(0.0, now - lastS_);
        lastS_ = now;

        const double groundH = scene.surfaceHeightM(targetLatRad_, targetLonRad_);
        const double targetH = std::isfinite(groundH) ? groundH : 0.0;
        double targetEcef[3];
        geodeticToEcef(targetLatRad_, targetLonRad_, targetH, targetEcef);

        double los[3];
        const double slantRangeM = geom::lineOfSight(own, targetEcef, los);
        const double horizonM = std::sqrt(2.0 * 6371000.0 * std::max(0.0, own.altitudeMslM)) + 5000.0;
        if (!(slantRangeM >= minRangeM_ && slantRangeM <= std::min(maxRangeM_, horizonM))) {
            collecting_ = false;
            finished_ = Reason::OutOfReach;
            return;
        }

        // Side-looking squint angle relative to aircraft velocity vector
        double velEcef[3];
        geom::velocityEcef(own, velEcef);
        const double speedMs = geom::norm3(velEcef);
        if (!(speedMs > 30.0)) {
            collecting_ = false;
            finished_ = Reason::OutOfReach;
            return;
        }
        double body[3];
        geom::toBody(own, los, body);
        geom::angles(body, beamAzRad_, beamElRad_);

        // Side-looking cross-track azimuth squint check: SAR cannot resolve cross-range
        // Doppler along the velocity ground track (|sin(az)| < 0.20, i.e., within ~11.5 deg of nose/tail).
        const double crossTrackSin = std::fabs(std::sin(beamAzRad_));
        if (crossTrackSin < 0.20) {
            collecting_ = false;
            finished_ = Reason::OutOfReach;
            return;
        }
        const double squintRad = std::asin(std::clamp(crossTrackSin, 0.20, 1.0));

        const double lambda = kLight / frequencyHz_;
        requiredIntS_ = std::clamp(integrationTimeS(lambda, slantRangeM, speedMs, resolutionM_, squintRad), 0.2, 60.0);
        geometryChecked_ = true;

        // Grazing / incidence angle at target
        const double dh = std::max(100.0, own.altitudeMslM - targetH);
        const double sinGrazing = std::clamp(dh / slantRangeM, 0.05, 0.99);
        const double incidenceRad = std::acos(sinGrazing);

        // Compute in-band RF jamming degradation against SAR receiver
        double totalJ0 = 0.0;
        for (const Emission& e : scene.emissions()) {
            if (e.mode != EmitterMode::Jammer || !e.state || e.state == &own) continue;
            const double overlap = ea::spectralOverlap(e.frequencyHz, e.bandwidthHz, frequencyHz_, bandwidthHz_);
            if (overlap <= 0.0) continue;
            const double txGain = e.gainTowardsDbi(own.positionEcef);
            const double rxGain = 0.0; // Sidelobe / off-beam average receiver coupling
            const double lossDb = scene.pathLossDb(e.entity, frequencyHz_);
            totalJ0 += ea::receivedJammerDensityWHz(e.powerW, txGain, rxGain, lossDb, e.bandwidthHz, overlap);
        }
        const double n0 = kBoltzmannT0 * std::pow(10.0, noiseFigureDb_ / 10.0);
        lastJamDegDb_ = (totalJ0 > 0.0) ? (10.0 * std::log10(1.0 + totalJ0 / n0)) : 0.0;

        lastNeszDb_ = neszDb(slantRangeM, speedMs, incidenceRad, avgPowerW_, antennaGainDbi_,
                             frequencyHz_, resolutionM_, noiseFigureDb_, systemLossDb_, lastJamDegDb_);
        accumIntS_ += dt;

        if (accumIntS_ + 1e-9 >= requiredIntS_) {
            Contact img;
            img.id = ++serial_;
            img.kind = ContactKind::Image;
            img.firstTimeS = startS_;
            img.timeS = now;
            img.azimuthRad = beamAzRad_;
            img.elevationRad = beamElRad_;
            img.bearingSigmaRad = 0.002;
            img.rangeM = slantRangeM;
            img.latitudeRad = targetLatRad_;
            img.longitudeRad = targetLonRad_;
            img.heightM = targetH;
            img.strengthDb = lastNeszDb_;
            img.sizeM = resolutionM_;
            images_.push_back(img);
            if (images_.size() > 32) images_.erase(images_.begin());
            collecting_ = false;
            finished_ = Reason::GoalReached;
        }
    }

    void idle(double now) override { stop(now); }

    void contacts(std::vector<Contact>& out) const override {
        for (const Contact& c : images_) out.push_back(c);
    }

    bool emission(Emission& out) const override {
        if (!collecting_ || !geometryChecked_) return false;
        out.mode = EmitterMode::Search;
        out.frequencyHz = frequencyHz_;
        out.bandwidthHz = bandwidthHz_;
        out.powerW = avgPowerW_;
        out.peakPowerW = peakPowerW_;
        out.peakGainDbi = antennaGainDbi_;
        out.sidelobeDbi = -8.0;
        out.azimuthCentreRad = beamAzRad_;
        out.azimuthHalfWidthRad = 3.0 * kDeg;
        out.elevationCentreRad = beamElRad_;
        out.elevationHalfWidthRad = 3.0 * kDeg;
        out.allRound = false;
        return true;
    }

    void report(std::vector<PayloadValue>& out) const override {
        out.push_back({"collecting", collecting_ ? 1.0 : 0.0, ""});
        out.push_back({"images", static_cast<double>(images_.size()), ""});
        out.push_back({"nesz", lastNeszDb_, "dB"});
        out.push_back({"resolution", resolutionM_, "m"});
        out.push_back({"integration_time", requiredIntS_, "s"});
        out.push_back({"jam_degradation", lastJamDegDb_, "dB"});
        out.push_back({"frequency", frequencyHz_, "Hz"});
    }

private:
    double frequencyHz_ = 9.6e9;
    double bandwidthHz_ = 300.0e6;
    double avgPowerW_ = 1200.0;
    double peakPowerW_ = 12000.0;
    double antennaGainDbi_ = 36.0;
    double noiseFigureDb_ = 4.0;
    double systemLossDb_ = 5.0;
    double finestResM_ = 0.3;
    double minRangeM_ = 15000.0;
    double maxRangeM_ = 200000.0;

    bool collecting_ = false;
    bool geometryChecked_ = false;
    Reason finished_ = Reason::None;
    double startS_ = 0.0;
    double lastS_ = 0.0;
    double targetLatRad_ = 0.0;
    double targetLonRad_ = 0.0;
    double resolutionM_ = 1.0;
    double requiredIntS_ = 1.0;
    double accumIntS_ = 0.0;
    double beamAzRad_ = 0.0;
    double beamElRad_ = 0.0;
    double lastNeszDb_ = std::numeric_limits<double>::quiet_NaN();
    double lastJamDegDb_ = 0.0;
    std::vector<Contact> images_;
    std::uint64_t serial_ = 0;
};

} // namespace

double integrationTimeS(double wavelengthM, double slantRangeM, double speedMs,
                        double azimuthResolutionM, double squintRad) noexcept {
    const double s = std::fabs(std::sin(squintRad));
    if (!(wavelengthM > 0.0) || !(slantRangeM > 0.0) || !(speedMs > 0.0) || !(azimuthResolutionM > 0.0) || !(s > 1e-6)) {
        return std::numeric_limits<double>::infinity();
    }
    return (wavelengthM * slantRangeM) / (2.0 * speedMs * azimuthResolutionM * s);
}

double neszDb(double slantRangeM, double speedMs, double incidenceRad,
              double avgPowerW, double antennaGainDbi, double frequencyHz,
              double rangeResolutionM, double noiseFigureDb, double systemLossDb,
              double extraJamDegradationDb) noexcept {
    if (!(slantRangeM > 0.0) || !(speedMs > 0.0) || !(avgPowerW > 0.0) || !(frequencyHz > 0.0) || !(rangeResolutionM > 0.0)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double lambda = kLight / frequencyHz;
    const double gLin = std::pow(10.0, antennaGainDbi / 10.0);
    const double nfLin = std::pow(10.0, noiseFigureDb / 10.0);
    const double lossLin = std::pow(10.0, systemLossDb / 10.0);
    const double sinInc = std::clamp(std::sin(incidenceRad), 0.05, 1.0);
    const double num = 4.0 * std::pow(4.0 * kPi, 3) * std::pow(slantRangeM, 3) * kBoltzmannT0 * nfLin * lossLin * speedMs * sinInc;
    const double den = avgPowerW * gLin * gLin * std::pow(lambda, 3) * rangeResolutionM;
    return 10.0 * std::log10(std::max(num / den, 1e-30)) + std::max(0.0, extraJamDegradationDb);
}

std::unique_ptr<Model> makeSarRadar(const Card& card, std::uint16_t count) {
    (void)count;
    return std::make_unique<SarRadar>(card);
}

} // namespace fsim::ms::sar
