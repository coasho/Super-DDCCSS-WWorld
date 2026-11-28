#include "simulation/mission_systems/ea/Jammer.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::ms::ea {

namespace {

constexpr double kPi = geom::kPi;
constexpr double kDeg = kPi / 180.0;

// Generic RF self-protection jammer fallback used only when a synthetic card omits parameters;
// all maintained electronic_attack cards declare their own parameters in assets/payloads/*.json.
struct ClassDefaults {
    double powerW = 400.0;
    double peakGainDbi = 10.0;
    double sidelobeDbi = -6.0;
    double bandMinHz = 2.0e9;
    double bandMaxHz = 18.0e9;
    double defaultAzHalfRad = 30.0 * kDeg;
    double defaultElHalfRad = 20.0 * kDeg;
    double spotBandwidthHz = 50.0e6;
    double minBandwidthHz = 1.0e6;
    bool drfmCapable = true;
    int maxFalseTargets = 6;
};

class Jammer final : public Model {
public:
    Jammer(const Card& card, const ClassDefaults& def, std::uint16_t count)
        : powerW_(card.value("power_w", def.powerW) * static_cast<double>(std::max<std::uint16_t>(1, count))),
          peakGainDbi_(card.value("peak_gain_dbi", def.peakGainDbi)),
          sidelobeDbi_(card.value("sidelobe_dbi", def.sidelobeDbi)),
          bandMinHz_(card.value("frequency_min_hz", def.bandMinHz)),
          bandMaxHz_(card.value("frequency_max_hz", def.bandMaxHz)),
          defaultAzHalfRad_(card.value("beam_azimuth_half_deg", def.defaultAzHalfRad / kDeg) * kDeg),
          defaultElHalfRad_(card.value("beam_elevation_half_deg", def.defaultElHalfRad / kDeg) * kDeg),
          spotBandwidthHz_(card.value("spot_bandwidth_hz", def.spotBandwidthHz)),
          minBandwidthHz_(card.value("min_bandwidth_hz", def.minBandwidthHz)),
          drfmCapable_(card.value("drfm_capable", def.drfmCapable ? 1.0 : 0.0) > 0.5),
          maxFalseTargets_(std::clamp(static_cast<int>(std::llround(card.value("max_false_targets", static_cast<double>(def.maxFalseTargets)))), 1, 8)) {}

    Reason begin(const PayloadTask& task, double now) override {
        if (task.kind != TaskKind::Jam) return Reason::InvalidParameter;
        if (std::isfinite(task.durationS) && !(task.durationS > 0.0)) return Reason::InvalidParameter;
        if (std::isfinite(task.bandwidthHz) && !(task.bandwidthHz >= minBandwidthHz_)) return Reason::InvalidParameter;
        if (std::isfinite(task.azimuthHalfWidthRad) && !(task.azimuthHalfWidthRad > 0.0)) return Reason::InvalidParameter;
        if (std::isfinite(task.elevationHalfWidthRad) && !(task.elevationHalfWidthRad > 0.0)) return Reason::InvalidParameter;

        double freq = task.frequencyHz;
        double bw = task.bandwidthHz;
        if (task.contact == 0) {
            if (std::isfinite(freq)) {
                if (freq < bandMinHz_ || freq > bandMaxHz_) return Reason::InvalidParameter;
                if (!std::isfinite(bw)) bw = spotBandwidthHz_;
            } else {
                // No contact and no explicit frequency: barrage across the jammer's full band.
                freq = 0.5 * (bandMinHz_ + bandMaxHz_);
                if (!std::isfinite(bw)) bw = std::max(minBandwidthHz_, bandMaxHz_ - bandMinHz_);
            }
        } else {
            // Reactive contact-following: frequency updated on first followed() callback.
            if (std::isfinite(freq) && (freq < bandMinHz_ || freq > bandMaxHz_)) return Reason::InvalidParameter;
            if (!std::isfinite(freq)) freq = 0.5 * (bandMinHz_ + bandMaxHz_);
            if (!std::isfinite(bw)) bw = spotBandwidthHz_;
        }

        const double azHalf = std::isfinite(task.azimuthHalfWidthRad)
                                  ? std::min(task.azimuthHalfWidthRad, kPi)
                                  : defaultAzHalfRad_;
        const double elHalf = std::isfinite(task.elevationHalfWidthRad)
                                  ? std::min(task.elevationHalfWidthRad, 0.5 * kPi)
                                  : defaultElHalfRad_;

        active_ = true;
        finished_ = Reason::None;
        startS_ = now;
        lastS_ = now;
        durationS_ = std::isfinite(task.durationS) ? task.durationS : std::numeric_limits<double>::quiet_NaN();
        followContact_ = task.contact;
        explicitFreq_ = std::isfinite(task.frequencyHz);
        contactAcquired_ = (task.contact == 0);
        frequencyHz_ = freq;
        bandwidthHz_ = std::max(minBandwidthHz_, bw);
        azCentreRad_ = std::isfinite(task.azimuthCentreRad) ? std::remainder(task.azimuthCentreRad, 2.0 * kPi) : 0.0;
        elCentreRad_ = std::isfinite(task.elevationCentreRad) ? std::clamp(task.elevationCentreRad, -0.5 * kPi, 0.5 * kPi) : 0.0;
        azHalfRad_ = azHalf;
        elHalfRad_ = elHalf;
        allRound_ = (azHalfRad_ >= kPi - 1e-6);
        deceptionMode_ = EaDeceptionModeV1::Noise;
        falseTargetCount_ = 0;
        rangeOffsetM_ = 0.0;
        rangePullRateMs_ = 0.0;
        rangePullMaxM_ = 0.0;
        velocityOffsetMs_ = 0.0;
        velocityPullRateMs2_ = 0.0;
        velocityPullMaxMs_ = 0.0;
        falseTargetRcsM2_ = std::numeric_limits<double>::quiet_NaN();
        return Reason::None;
    }

    Reason beginDeception(const EaDeceptionCommandV1& command, double now) override {
        if (command.version != 1) return Reason::InvalidParameter;
        if (static_cast<unsigned>(command.mode) >= static_cast<unsigned>(EaDeceptionModeV1::Count)) {
            return Reason::InvalidParameter;
        }
        if (command.mode != EaDeceptionModeV1::Noise && !drfmCapable_) return Reason::NotSupported;
        if (!std::isfinite(command.rangeOffsetM) || command.rangeOffsetM < 0.0) return Reason::InvalidParameter;
        if (!std::isfinite(command.rangePullRateMs) || command.rangePullRateMs < 0.0) return Reason::InvalidParameter;
        if (!std::isfinite(command.rangePullMaxM) || command.rangePullMaxM < 0.0) return Reason::InvalidParameter;
        if (!std::isfinite(command.velocityOffsetMs) || !std::isfinite(command.velocityPullRateMs2) ||
            !std::isfinite(command.velocityPullMaxMs) || command.velocityPullMaxMs < 0.0) {
            return Reason::InvalidParameter;
        }
        if (std::isfinite(command.falseTargetRcsM2) && !(command.falseTargetRcsM2 > 0.0)) {
            return Reason::InvalidParameter;
        }
        if (command.mode == EaDeceptionModeV1::FalseTargets &&
            (command.falseTargetCount < 1 || command.falseTargetCount > maxFalseTargets_)) {
            return Reason::InvalidParameter;
        }
        PayloadTask base;
        base.payload = command.payload;
        base.kind = TaskKind::Jam;
        base.contact = command.contact;
        base.azimuthCentreRad = command.azimuthCentreRad;
        base.elevationCentreRad = command.elevationCentreRad;
        base.azimuthHalfWidthRad = command.azimuthHalfWidthRad;
        base.elevationHalfWidthRad = command.elevationHalfWidthRad;
        base.frequencyHz = command.frequencyHz;
        base.bandwidthHz = command.bandwidthHz;
        base.durationS = command.durationS;
        const Reason br = begin(base, now);
        if (br != Reason::None) return br;
        deceptionMode_ = command.mode;
        falseTargetCount_ = (command.mode == EaDeceptionModeV1::FalseTargets) ? command.falseTargetCount
                            : (command.mode == EaDeceptionModeV1::Noise ? 0 : 1);
        rangeOffsetM_ = command.rangeOffsetM;
        rangePullRateMs_ = command.rangePullRateMs;
        rangePullMaxM_ = command.rangePullMaxM;
        velocityOffsetMs_ = command.velocityOffsetMs;
        velocityPullRateMs2_ = command.velocityPullRateMs2;
        velocityPullMaxMs_ = command.velocityPullMaxMs;
        falseTargetRcsM2_ = command.falseTargetRcsM2;
        return Reason::None;
    }

    void stop(double) override {
        active_ = false;
        followContact_ = 0;
        contactAcquired_ = false;
        finished_ = Reason::None;
    }

    Reason finished() const override { return finished_; }

    double percent() const override {
        if (!active_ || !std::isfinite(durationS_) || durationS_ <= 0.0) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return std::clamp(100.0 * (lastS_ - startS_) / durationS_, 0.0, 100.0);
    }

    std::uint64_t follows() const override { return active_ ? followContact_ : 0; }

    void followed(const Contact* held) override {
        if (!active_ || followContact_ == 0) return;
        if (!held) {
            active_ = false;
            contactAcquired_ = false;
            finished_ = Reason::ContactLost;
            return;
        }
        if (std::isfinite(held->frequencyHz) && !explicitFreq_) {
            if (held->frequencyHz < bandMinHz_ || held->frequencyHz > bandMaxHz_) {
                active_ = false;
                contactAcquired_ = false;
                finished_ = Reason::ContactLost;
                return;
            }
            frequencyHz_ = held->frequencyHz;
        }
        azCentreRad_ = held->azimuthRad;
        elCentreRad_ = held->elevationRad;
        allRound_ = false;
        contactAcquired_ = true;
    }

    void step(double now, const sim::VehicleState&, const Scene&, Rng&) override {
        if (!active_) return;
        lastS_ = now;
        if (std::isfinite(durationS_) && now - startS_ + 1e-9 >= durationS_) {
            active_ = false;
            finished_ = Reason::GoalReached;
        }
    }

    void idle(double now) override { stop(now); }

    bool emission(Emission& out) const override {
        if (!active_ || !contactAcquired_) return false;
        out.mode = EmitterMode::Jammer;
        out.frequencyHz = frequencyHz_;
        out.bandwidthHz = bandwidthHz_;
        out.powerW = powerW_;
        out.peakPowerW = 0.0; // Continuous noise jamming: average power equals instantaneous power.
        out.peakGainDbi = allRound_ ? std::min(peakGainDbi_, 3.0) : peakGainDbi_;
        out.sidelobeDbi = sidelobeDbi_;
        out.azimuthCentreRad = azCentreRad_;
        out.azimuthHalfWidthRad = azHalfRad_;
        out.elevationCentreRad = elCentreRad_;
        out.elevationHalfWidthRad = elHalfRad_;
        out.allRound = allRound_;
        out.target = 0;
        out.deceptionMode = deceptionMode_;
        out.deceptionElapsedS = std::max(0.0, lastS_ - startS_);
        out.falseTargetCount = falseTargetCount_;
        out.rangeOffsetM = rangeOffsetM_;
        out.rangePullRateMs = rangePullRateMs_;
        out.rangePullMaxM = rangePullMaxM_;
        out.velocityOffsetMs = velocityOffsetMs_;
        out.velocityPullRateMs2 = velocityPullRateMs2_;
        out.velocityPullMaxMs = velocityPullMaxMs_;
        out.falseTargetRcsM2 = falseTargetRcsM2_;
        return true;
    }

    void report(std::vector<PayloadValue>& out) const override {
        const bool radiating = active_ && contactAcquired_;
        const double gain = allRound_ ? std::min(peakGainDbi_, 3.0) : peakGainDbi_;
        const double erpDbw = radiating ? (10.0 * std::log10(std::max(powerW_, 1e-12)) + gain) : -std::numeric_limits<double>::infinity();
        out.push_back({"active", radiating ? 1.0 : 0.0, ""});
        out.push_back({"power", radiating ? powerW_ : 0.0, "W"});
        out.push_back({"frequency", radiating ? frequencyHz_ : std::numeric_limits<double>::quiet_NaN(), "Hz"});
        out.push_back({"bandwidth", radiating ? bandwidthHz_ : std::numeric_limits<double>::quiet_NaN(), "Hz"});
        out.push_back({"erp", radiating ? erpDbw : std::numeric_limits<double>::quiet_NaN(), "dBW"});
        out.push_back({"band_min", bandMinHz_, "Hz"});
        out.push_back({"band_max", bandMaxHz_, "Hz"});
        out.push_back({"deception_mode", radiating ? static_cast<double>(deceptionMode_) : 0.0, ""});
        out.push_back({"false_targets", radiating ? static_cast<double>(falseTargetCount_) : 0.0, ""});
    }

private:
    double powerW_ = 400.0;
    double peakGainDbi_ = 10.0;
    double sidelobeDbi_ = -6.0;
    double bandMinHz_ = 2.0e9;
    double bandMaxHz_ = 18.0e9;
    double defaultAzHalfRad_ = 30.0 * kDeg;
    double defaultElHalfRad_ = 20.0 * kDeg;
    double spotBandwidthHz_ = 50.0e6;
    double minBandwidthHz_ = 1.0e6;
    bool drfmCapable_ = true;
    int maxFalseTargets_ = 6;

    bool active_ = false;
    bool contactAcquired_ = false;
    bool explicitFreq_ = false;
    bool allRound_ = false;
    Reason finished_ = Reason::None;
    double startS_ = 0.0;
    double lastS_ = 0.0;
    double durationS_ = std::numeric_limits<double>::quiet_NaN();
    std::uint64_t followContact_ = 0;
    double frequencyHz_ = 10.0e9;
    double bandwidthHz_ = 50.0e6;
    double azCentreRad_ = 0.0;
    double elCentreRad_ = 0.0;
    double azHalfRad_ = 30.0 * kDeg;
    double elHalfRad_ = 20.0 * kDeg;
    EaDeceptionModeV1 deceptionMode_ = EaDeceptionModeV1::Noise;
    int falseTargetCount_ = 0;
    double rangeOffsetM_ = 0.0;
    double rangePullRateMs_ = 0.0;
    double rangePullMaxM_ = 0.0;
    double velocityOffsetMs_ = 0.0;
    double velocityPullRateMs2_ = 0.0;
    double velocityPullMaxMs_ = 0.0;
    double falseTargetRcsM2_ = std::numeric_limits<double>::quiet_NaN();
};

} // namespace

double spectralOverlap(double jamFreqHz, double jamBwHz, double rxFreqHz, double rxBwHz) noexcept {
    if (!(jamBwHz > 0.0) || !(rxBwHz > 0.0) || !std::isfinite(jamFreqHz) || !std::isfinite(rxFreqHz)) {
        return 0.0;
    }
    const double jLo = jamFreqHz - 0.5 * jamBwHz;
    const double jHi = jamFreqHz + 0.5 * jamBwHz;
    const double rLo = rxFreqHz - 0.5 * rxBwHz;
    const double rHi = rxFreqHz + 0.5 * rxBwHz;
    const double lo = std::max(jLo, rLo);
    const double hi = std::min(jHi, rHi);
    if (!(hi > lo)) return 0.0;
    return std::clamp((hi - lo) / rxBwHz, 0.0, 1.0);
}

double receivedJammerDensityWHz(double powerW, double txGainDbi, double rxGainDbi,
                                double pathLossDb, double jamBwHz, double overlap) noexcept {
    if (!(powerW > 0.0) || !(jamBwHz > 0.0) || !(overlap > 0.0) ||
        !std::isfinite(txGainDbi) || !std::isfinite(rxGainDbi) || !std::isfinite(pathLossDb)) {
        return 0.0;
    }
    const double netGainDb = txGainDbi + rxGainDbi - pathLossDb;
    const double linearGain = std::pow(10.0, netGainDb / 10.0);
    return (powerW * linearGain / jamBwHz) * overlap;
}

void evaluateDrfmGhosts(const Emission& em, double rangeM, double victimPowerW,
                        double victimGainLinear, double victimPathExcessLinear,
                        double txGainDbi, double overlap,
                        std::vector<DrfmGhost>& out) noexcept {
    if (em.deceptionMode == EaDeceptionModeV1::Noise || !(em.powerW > 0.0) || !(overlap > 0.0) ||
        !(rangeM > 1.0) || !(victimPowerW > 0.0) || !(victimGainLinear > 0.0) ||
        !std::isfinite(txGainDbi) || !std::isfinite(victimPathExcessLinear)) {
        return;
    }
    const int nGhosts = (em.deceptionMode == EaDeceptionModeV1::FalseTargets)
                            ? std::clamp(em.falseTargetCount, 1, 8)
                            : 1;
    const double txGainLin = std::pow(10.0, txGainDbi / 10.0);
    const double erpPerGhostW = (em.powerW * txGainLin / static_cast<double>(nGhosts)) * overlap;
    const double denom = std::max(victimPowerW * victimGainLinear, 1e-6);
    // Exact one-way DRFM repeater vs two-way passive skin echo equivalence:
    //   P_r_drfm = (ERP / N) * G_rx * lambda^2 / ((4*pi)^2 * R^2 * L_excess)
    //   P_r_real = P_tx * G_rx^2 * sigma * lambda^2 / ((4*pi)^3 * R^4 * L_excess^2)
    //   => sigma_eq_max = 4*pi * R^2 * L_excess * (ERP / N) / (P_tx * G_rx).
    const double sigmaErpCapM2 =
        (4.0 * kPi * rangeM * rangeM * std::max(victimPathExcessLinear, 1.0) * erpPerGhostW) / denom;
    const double targetRcs =
        (std::isfinite(em.falseTargetRcsM2) && em.falseTargetRcsM2 > 0.0)
            ? em.falseTargetRcsM2
            : (em.deceptionMode == EaDeceptionModeV1::FalseTargets ? 50.0 : 80.0);
    const double sigmaEq = std::min(sigmaErpCapM2, targetRcs);
    if (!(sigmaEq > 1e-4) || !std::isfinite(sigmaEq)) return;

    const double elapsed = std::max(0.0, std::isfinite(em.deceptionElapsedS) ? em.deceptionElapsedS : 0.0);
    const double baseRange = std::max(0.0, std::isfinite(em.rangeOffsetM) ? em.rangeOffsetM : 0.0);
    const double baseVel = std::isfinite(em.velocityOffsetMs) ? em.velocityOffsetMs : 0.0;
    constexpr double kHoldS = 1.0;
    constexpr double kBlankS = 0.6;

    switch (em.deceptionMode) {
    case EaDeceptionModeV1::Rgpo: {
        const double walkRate = std::max(10.0, em.rangePullRateMs);
        const double maxPull = std::max(200.0, em.rangePullMaxM);
        const double walkS = maxPull / walkRate;
        const double cycleS = kHoldS + walkS + kBlankS;
        const double tCyc = std::fmod(elapsed, cycleS);
        if (tCyc >= kHoldS + walkS) return; // Blanking drop-off phase
        const double walkT = std::max(0.0, tCyc - kHoldS);
        out.push_back({0, baseRange + walkRate * walkT, baseVel, sigmaEq});
        break;
    }
    case EaDeceptionModeV1::Vgpo: {
        const double accel = std::max(5.0, std::fabs(em.velocityPullRateMs2));
        const double sign = em.velocityPullRateMs2 < 0.0 ? -1.0 : 1.0;
        const double maxV = std::max(20.0, std::fabs(em.velocityPullMaxMs));
        const double walkS = maxV / accel;
        const double cycleS = kHoldS + walkS + kBlankS;
        const double tCyc = std::fmod(elapsed, cycleS);
        if (tCyc >= kHoldS + walkS) return;
        const double walkT = std::max(0.0, tCyc - kHoldS);
        out.push_back({0, baseRange, baseVel + sign * accel * walkT, sigmaEq});
        break;
    }
    case EaDeceptionModeV1::RgpoVgpo: {
        const double walkRate = std::max(10.0, em.rangePullRateMs);
        const double maxPull = std::max(200.0, em.rangePullMaxM);
        const double accel = std::max(5.0, std::fabs(em.velocityPullRateMs2));
        const double sign = em.velocityPullRateMs2 < 0.0 ? -1.0 : 1.0;
        const double maxV = std::max(20.0, std::fabs(em.velocityPullMaxMs));
        const double walkS = std::max(maxPull / walkRate, maxV / accel);
        const double cycleS = kHoldS + walkS + kBlankS;
        const double tCyc = std::fmod(elapsed, cycleS);
        if (tCyc >= kHoldS + walkS) return;
        const double walkT = std::max(0.0, tCyc - kHoldS);
        out.push_back({0,
                       baseRange + std::min(maxPull, walkRate * walkT),
                       baseVel + sign * std::min(maxV, accel * walkT),
                       sigmaEq});
        break;
    }
    case EaDeceptionModeV1::FalseTargets: {
        const double spacing = (baseRange > 50.0) ? baseRange : 1800.0;
        for (int k = 0; k < nGhosts; ++k) {
            out.push_back({k, spacing * static_cast<double>(k + 1), baseVel + static_cast<double>(k) * 8.0, sigmaEq});
        }
        break;
    }
    default:
        break;
    }
}

std::unique_ptr<Model> makeJammer(const Card& card, std::uint16_t count) {
    if (card.value("is_dircm", 0.0) > 0.5) return nullptr;
    return std::make_unique<Jammer>(card, ClassDefaults{}, count);
}

} // namespace fsim::ms::ea
