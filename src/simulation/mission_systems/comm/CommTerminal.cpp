#include "simulation/mission_systems/comm/CommTerminal.h"
#include "simulation/mission_systems/ea/Jammer.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::ms::comm_payload {

namespace {

constexpr double kBoltzmannT0 = 1.380649e-23 * 290.0;

struct CommClassDefaults {
    double bandMinHz = 969.0e6;
    double bandMaxHz = 1206.0e6;
    double channelBwHz = 3.0e6;
    double processingGainDb = 17.0; // Link-16 51-channel FHSS processing gain (~17 dB)
    double jamThresholdJToNDb = 25.0;
    double rxGainDbi = 0.0;
    bool iffCapable = false;
    bool iffMode5 = false;
};

CommClassDefaults defaultsFor(const Card& card) {
    CommClassDefaults d;
    const std::string& id = card.id;
    if (id == "mids-jtrs") {
        d.bandMinHz = 969.0e6;
        d.bandMaxHz = 1206.0e6;
        d.processingGainDb = 17.0;
        d.iffCapable = true;
        d.iffMode5 = true;
    } else if (id == "an-asq-242") {
        // Integrated CNI: MADL Ku-band directional + Link-16 + Mode 5 IFF
        d.bandMinHz = 969.0e6;
        d.bandMaxHz = 15.0e9;
        d.processingGainDb = 24.0; // Directional MADL + FHSS AJ
        d.iffCapable = true;
        d.iffMode5 = true;
    } else if (id == "ifdl") {
        // Directional Ku-band LPI intra-flight datalink
        d.bandMinHz = 14.4e9;
        d.bandMaxHz = 15.35e9;
        d.channelBwHz = 50.0e6;
        d.processingGainDb = 26.0;
    } else if (id == "an-arc-210") {
        // VHF/UHF software radio (30-512 MHz)
        d.bandMinHz = 30.0e6;
        d.bandMaxHz = 512.0e6;
        d.channelBwHz = 25.0e3;
        d.processingGainDb = 6.0;
    } else if (id == "u2s-datalinks") {
        d.bandMinHz = 10.0e9;
        d.bandMaxHz = 15.0e9;
        d.channelBwHz = 100.0e6;
        d.processingGainDb = 15.0;
    } else {
        // Link-16 / JTIDS / MIDS (`mids`, `jtids-2h`, `a10c-link16`, `gripen-link16`)
        d.bandMinHz = 969.0e6;
        d.bandMaxHz = 1206.0e6;
        d.processingGainDb = 17.0;
    }
    return d;
}

class CommTerminal final : public Model {
public:
    explicit CommTerminal(const Card& card) {
        const CommClassDefaults def = defaultsFor(card);
        bandMinHz_ = card.value("frequency_min_hz", def.bandMinHz);
        bandMaxHz_ = card.value("frequency_max_hz", def.bandMaxHz);
        processingGainDb_ = card.value("processing_gain_db", def.processingGainDb);
        jamThresholdJToNDb_ = card.value("jam_threshold_j_to_n_db", def.jamThresholdJToNDb);
        rxGainDbi_ = card.value("antenna_gain_dbi", def.rxGainDbi);
        iffCapable_ = card.value("iff_capable", def.iffCapable ? 1.0 : 0.0) > 0.5;
        iffMode5_ = card.value("iff_mode5", def.iffMode5 ? 1.0 : 0.0) > 0.5;
    }

    void step(double, const sim::VehicleState& own, const Scene& scene, Rng&) override {
        active_ = true;
        const double centerHz = 0.5 * (bandMinHz_ + bandMaxHz_);
        const double bandBwHz = std::max(1.0e6, bandMaxHz_ - bandMinHz_);
        double totalJ0 = 0.0;
        for (const Emission& e : scene.emissions()) {
            if (e.mode != EmitterMode::Jammer || !e.state || e.state == &own) continue;
            const double overlap = ea::spectralOverlap(e.frequencyHz, e.bandwidthHz, centerHz, bandBwHz);
            if (overlap <= 0.0) continue;
            const double txGain = e.gainTowardsDbi(own.positionEcef);
            const double lossDb = scene.pathLossDb(e.entity, std::clamp(e.frequencyHz, bandMinHz_, bandMaxHz_));
            totalJ0 += ea::receivedJammerDensityWHz(e.powerW, txGain, rxGainDbi_, lossDb, e.bandwidthHz, overlap);
        }
        const double n0 = kBoltzmannT0 * std::pow(10.0, 5.0 / 10.0);
        const double jamPowerW = totalJ0 * bandBwHz;
        jamPowerDbm_ = (jamPowerW > 0.0) ? (10.0 * std::log10(jamPowerW * 1000.0)) : -std::numeric_limits<double>::infinity();
        const double rawJToNDb = (totalJ0 > 0.0) ? (10.0 * std::log10(totalJ0 / n0)) : -std::numeric_limits<double>::infinity();
        effJToNDb_ = std::isfinite(rawJToNDb) ? (rawJToNDb - processingGainDb_) : -std::numeric_limits<double>::infinity();
        jammed_ = std::isfinite(effJToNDb_) && (effJToNDb_ > jamThresholdJToNDb_);

        // Mode 5 IFF transponder link count (active friendly/cooperative interrogable aircraft within 1090 MHz horizon)
        iffInRangeCount_ = 0;
        if (iffCapable_) {
            std::uint32_t ids[512];
            const sim::VehicleState* targets[512];
            const std::size_t n = scene.targets(ids, targets, 512);
            for (std::size_t k = 0; k < n; ++k) {
                if (!targets[k] || scene.role(ids[k]) != Role::Aircraft) continue;
                double los[3];
                const double r = geom::lineOfSight(own, targets[k]->positionEcef, los);
                if (!(r > 1.0) || scene.masked(ids[k], r)) continue;
                const double lossDb = scene.pathLossDb(ids[k], kIffReplyHz);
                // Nominal Mode 5 transponder 500 W (27 dBW) ERP, -80 dBm interrogator sensitivity
                if (iffLinkMarginDb(27.0, lossDb, 0.0, -80.0) >= 0.0) {
                    ++iffInRangeCount_;
                }
            }
        }
    }

    void idle(double) override {
        active_ = false;
        jammed_ = false;
        iffInRangeCount_ = 0;
    }

    void report(std::vector<PayloadValue>& out) const override {
        out.push_back({"active", active_ ? 1.0 : 0.0, ""});
        out.push_back({"link_available", (active_ && !jammed_) ? 1.0 : 0.0, ""});
        out.push_back({"jammed", jammed_ ? 1.0 : 0.0, ""});
        out.push_back({"jam_power", std::isfinite(jamPowerDbm_) ? jamPowerDbm_ : std::numeric_limits<double>::quiet_NaN(), "dBm"});
        out.push_back({"j_to_n", std::isfinite(effJToNDb_) ? effJToNDb_ : std::numeric_limits<double>::quiet_NaN(), "dB"});
        out.push_back({"processing_gain", processingGainDb_, "dB"});
        out.push_back({"band_min", bandMinHz_, "Hz"});
        out.push_back({"band_max", bandMaxHz_, "Hz"});
        out.push_back({"iff_capable", iffCapable_ ? 1.0 : 0.0, ""});
        out.push_back({"iff_mode5", iffMode5_ ? 1.0 : 0.0, ""});
        out.push_back({"iff_in_range", static_cast<double>(iffInRangeCount_), ""});
    }

private:
    double bandMinHz_ = 969.0e6;
    double bandMaxHz_ = 1206.0e6;
    double processingGainDb_ = 17.0;
    double jamThresholdJToNDb_ = 25.0;
    double rxGainDbi_ = 0.0;
    bool iffCapable_ = false;
    bool iffMode5_ = false;

    bool active_ = false;
    bool jammed_ = false;
    double jamPowerDbm_ = -std::numeric_limits<double>::infinity();
    double effJToNDb_ = -std::numeric_limits<double>::infinity();
    std::size_t iffInRangeCount_ = 0;
};

} // namespace

double iffLinkMarginDb(double txErpDbw, double pathLossDb, double rxGainDbi, double rxSensitivityDbm) noexcept {
    const double rxDbm = (txErpDbw + 30.0) - pathLossDb + rxGainDbi;
    return rxDbm - rxSensitivityDbm;
}

std::unique_ptr<Model> makeCommTerminal(const Card& card, std::uint16_t count) {
    (void)count;
    return std::make_unique<CommTerminal>(card);
}

} // namespace fsim::ms::comm_payload
