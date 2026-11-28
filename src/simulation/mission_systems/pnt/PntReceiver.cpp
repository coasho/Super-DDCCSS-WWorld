#include "simulation/mission_systems/pnt/PntReceiver.h"
#include "simulation/mission_systems/ea/Jammer.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::ms::pnt {

namespace {

constexpr double kPi = geom::kPi;
constexpr double kBoltzmannT0 = 1.380649e-23 * 290.0;
constexpr double kGpsL1Hz = 1.57542e9;
constexpr double kGpsL2Hz = 1.22760e9;
constexpr double kGpsBwHz = 20.46e6; ///< P(Y)/M-code military spread bandwidth

class PntReceiver final : public Model {
public:
    explicit PntReceiver(const Card& card) {
        // Generic pure-INS fallback when a synthetic card omits parameters;
        // maintained pnt cards declare their own parameters in assets/payloads/*.json.
        gnssCapable_ = card.value("gnss_capable", 0.0) > 0.5;
        driftRateMs_ = card.value("drift_rate_m_s", 0.412); // ~0.8 nmi/h navigation-grade INS
        nominalHorizM_ = card.value("nominal_horizontal_m", 20.0);
        nominalVertM_ = card.value("nominal_vertical_m", 15.0);
        carrierDbw_ = card.value("gnss_carrier_dbw", -160.0); // IS-GPS-200 -160 dBW P(Y) min
        noiseFigureDb_ = card.value("noise_figure_db", 3.0);
        antiJamDb_ = card.value("anti_jam_db", 24.0);
        lockThresholdDbHz_ = card.value("lock_threshold_dbhz", 22.0);
        horizErrorM_ = nominalHorizM_;
        vertErrorM_ = nominalVertM_;
        gnssLocked_ = gnssCapable_;
        cn0DbHz_ = gnssCapable_ ? effectiveGnssCn0DbHz(carrierDbw_, noiseFigureDb_, 0.0, antiJamDb_)
                                : std::numeric_limits<double>::quiet_NaN();
    }

    void step(double now, const sim::VehicleState& own, const Scene& scene, Rng&) override {
        if (!started_) {
            started_ = true;
            startS_ = now;
            lastS_ = now;
        }
        const double dt = std::max(0.0, now - lastS_);
        lastS_ = now;
        uptimeS_ = std::max(0.0, now - startS_);

        if (!gnssCapable_) {
            // Pure INS (an-asn-109): immune to RF jamming; grows with Schuler oscillation + drift rate
            jammed_ = false;
            gnssLocked_ = false;
            cn0DbHz_ = std::numeric_limits<double>::quiet_NaN();
            jToSDb_ = -std::numeric_limits<double>::infinity();
            horizErrorM_ = insHorizontalErrorM(uptimeS_, driftRateMs_, nominalHorizM_);
            vertErrorM_ = nominalVertM_ + 0.02 * uptimeS_;
            velErrorMs_ = 0.5 + 0.0005 * uptimeS_;
            return;
        }

        // Military SAASM GPS (magr-2k): evaluate live RF jammer density across L1 and L2
        double j0L1 = 0.0;
        double j0L2 = 0.0;
        for (const Emission& e : scene.emissions()) {
            if (e.mode != EmitterMode::Jammer || !e.state || e.state == &own ||
                e.deceptionMode != EaDeceptionModeV1::Noise) continue;
            const double ov1 = ea::spectralOverlap(e.frequencyHz, e.bandwidthHz, kGpsL1Hz, kGpsBwHz);
            const double ov2 = ea::spectralOverlap(e.frequencyHz, e.bandwidthHz, kGpsL2Hz, kGpsBwHz);
            if (ov1 <= 0.0 && ov2 <= 0.0) continue;
            const double txGain = e.gainTowardsDbi(own.positionEcef);
            const double rxGain = 0.0; // Hemispherical zenith GPS antenna gain
            if (ov1 > 0.0) {
                const double loss1 = scene.pathLossDb(e.entity, kGpsL1Hz);
                j0L1 += ea::receivedJammerDensityWHz(e.powerW, txGain, rxGain, loss1, e.bandwidthHz, ov1);
            }
            if (ov2 > 0.0) {
                const double loss2 = scene.pathLossDb(e.entity, kGpsL2Hz);
                j0L2 += ea::receivedJammerDensityWHz(e.powerW, txGain, rxGain, loss2, e.bandwidthHz, ov2);
            }
        }
        // Dual-frequency SAASM tracks the cleaner of L1 or L2
        const double bestJ0 = std::min(j0L1, j0L2);
        cn0DbHz_ = effectiveGnssCn0DbHz(carrierDbw_, noiseFigureDb_, bestJ0, antiJamDb_);
        const double carrierW = std::pow(10.0, carrierDbw_ / 10.0);
        jToSDb_ = (bestJ0 > 0.0) ? (10.0 * std::log10(bestJ0 * kGpsBwHz / std::max(carrierW, 1e-30)))
                                 : -std::numeric_limits<double>::infinity();

        if (cn0DbHz_ < lockThresholdDbHz_) {
            jammed_ = true;
            gnssLocked_ = false;
            deniedS_ += dt;
            horizErrorM_ = nominalHorizM_ + driftRateMs_ * deniedS_;
            vertErrorM_ = nominalVertM_ + 0.15 * deniedS_;
            velErrorMs_ = 0.2 + 0.01 * deniedS_;
        } else {
            jammed_ = false;
            gnssLocked_ = true;
            deniedS_ = 0.0;
            const double degrade = std::max(1.0, std::pow(10.0, std::max(0.0, 42.0 - cn0DbHz_) / 20.0));
            horizErrorM_ = nominalHorizM_ * degrade;
            vertErrorM_ = nominalVertM_ * degrade;
            velErrorMs_ = 0.05 * degrade;
        }
    }

    void idle(double) override {
        started_ = false;
        uptimeS_ = 0.0;
        deniedS_ = 0.0;
        jammed_ = false;
        gnssLocked_ = false;
    }

    bool pntSolution(PntSolution& out) const override {
        if (!started_) return false;
        out.active = true;
        out.gnssCapable = gnssCapable_;
        out.gnssLocked = gnssLocked_;
        out.jammed = jammed_;
        out.horizErrorM = horizErrorM_;
        out.vertErrorM = vertErrorM_;
        out.velErrorMs = velErrorMs_;
        return true;
    }

    void report(std::vector<PayloadValue>& out) const override {
        out.push_back({"valid", started_ ? 1.0 : 0.0, ""});
        out.push_back({"gnss_capable", gnssCapable_ ? 1.0 : 0.0, ""});
        out.push_back({"gnss_locked", gnssLocked_ ? 1.0 : 0.0, ""});
        out.push_back({"jammed", jammed_ ? 1.0 : 0.0, ""});
        out.push_back({"cn0", cn0DbHz_, "dB-Hz"});
        out.push_back({"j_to_s", std::isfinite(jToSDb_) ? jToSDb_ : std::numeric_limits<double>::quiet_NaN(), "dB"});
        out.push_back({"pos_error_h", horizErrorM_, "m"});
        out.push_back({"pos_error_v", vertErrorM_, "m"});
        out.push_back({"vel_error", velErrorMs_, "m/s"});
        out.push_back({"drift_rate", driftRateMs_, "m/s"});
        out.push_back({"uptime", uptimeS_, "s"});
    }

private:
    bool gnssCapable_ = false;
    double driftRateMs_ = 0.412;
    double nominalHorizM_ = 20.0;
    double nominalVertM_ = 15.0;
    double carrierDbw_ = -160.0;
    double noiseFigureDb_ = 3.0;
    double antiJamDb_ = 24.0;
    double lockThresholdDbHz_ = 22.0;

    bool started_ = false;
    bool gnssLocked_ = false;
    bool jammed_ = false;
    double startS_ = 0.0;
    double lastS_ = 0.0;
    double uptimeS_ = 0.0;
    double deniedS_ = 0.0;
    double cn0DbHz_ = std::numeric_limits<double>::quiet_NaN();
    double jToSDb_ = -std::numeric_limits<double>::infinity();
    double horizErrorM_ = 20.0;
    double vertErrorM_ = 15.0;
    double velErrorMs_ = 0.5;
};

} // namespace

double effectiveGnssCn0DbHz(double carrierPowerDbw, double noiseFigureDb,
                            double jammerDensityWHz, double antiJamMarginDb) noexcept {
    const double cW = std::pow(10.0, carrierPowerDbw / 10.0);
    const double n0 = kBoltzmannT0 * std::pow(10.0, noiseFigureDb / 10.0);
    const double ajLin = std::pow(10.0, std::max(0.0, antiJamMarginDb) / 10.0);
    const double effNoise = n0 + std::max(0.0, jammerDensityWHz) / ajLin;
    return 10.0 * std::log10(std::max(cW / std::max(effNoise, 1e-35), 1e-12));
}

double insHorizontalErrorM(double elapsedS, double driftRateMs, double initialErrorM) noexcept {
    const double t = std::max(0.0, elapsedS);
    const double schuler = 0.35 * driftRateMs * (kSchulerPeriodS / (2.0 * kPi)) * std::fabs(std::sin(2.0 * kPi * t / kSchulerPeriodS));
    return std::max(0.0, initialErrorM) + driftRateMs * t + schuler;
}

std::unique_ptr<Model> makePntReceiver(const Card& card, std::uint16_t count) {
    (void)count;
    return std::make_unique<PntReceiver>(card);
}

} // namespace fsim::ms::pnt
