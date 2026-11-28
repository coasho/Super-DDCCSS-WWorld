#include "simulation/mission_systems/comm/CommTerminal.h"
#include "simulation/mission_systems/ea/Jammer.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::ms::comm_payload {

namespace {

constexpr double kBoltzmannT0 = 1.380649e-23 * 290.0;
constexpr double kPi = geom::kPi;
constexpr double kLight = 299792458.0;
constexpr double kIffBwHz = 3.0e6;

struct CommClassDefaults {
    double bandMinHz = 969.0e6;
    double bandMaxHz = 1206.0e6;
    double txPowerW = 200.0;
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
        d.txPowerW = 200.0;
        d.processingGainDb = 17.0;
        d.iffCapable = true;
        d.iffMode5 = true;
    } else if (id == "an-asq-242") {
        // Integrated CNI: MADL Ku-band directional + Link-16 + Mode 5 IFF
        d.bandMinHz = 969.0e6;
        d.bandMaxHz = 15.0e9;
        d.txPowerW = 100.0;
        d.processingGainDb = 24.0; // Directional MADL + FHSS AJ
        d.iffCapable = true;
        d.iffMode5 = true;
    } else if (id == "ifdl") {
        // Directional Ku-band LPI intra-flight datalink
        d.bandMinHz = 14.4e9;
        d.bandMaxHz = 15.35e9;
        d.txPowerW = 50.0;
        d.channelBwHz = 50.0e6;
        d.processingGainDb = 26.0;
    } else if (id == "an-arc-210") {
        // VHF/UHF software radio (30-512 MHz)
        d.bandMinHz = 30.0e6;
        d.bandMaxHz = 512.0e6;
        d.txPowerW = 25.0;
        d.channelBwHz = 25.0e3;
        d.processingGainDb = 6.0;
    } else if (id == "u2s-datalinks") {
        d.bandMinHz = 10.0e9;
        d.bandMaxHz = 15.0e9;
        d.txPowerW = 150.0;
        d.channelBwHz = 100.0e6;
        d.processingGainDb = 15.0;
    } else {
        // Link-16 / JTIDS / MIDS (`mids`, `jtids-2h`, `a10c-link16`, `gripen-link16`)
        d.bandMinHz = 969.0e6;
        d.bandMaxHz = 1206.0e6;
        d.txPowerW = 200.0;
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
        txPowerW_ = card.value("tx_power_w", def.txPowerW);
        channelBwHz_ = card.value("channel_bandwidth_hz", def.channelBwHz);
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
            if (e.mode != EmitterMode::Jammer || !e.state || e.state == &own ||
                e.deceptionMode != EaDeceptionModeV1::Noise) continue;
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

        // Two-way STANAG 4193 / ICAO Annex 10 Mode 5 IFF:
        // - Checks cooperative transponder state on target
        // - Evaluates 1030 MHz interrogation uplink (30 dBW interrogator ERP, -77 dBm transponder MTL + 1030 MHz jamming)
        // - Evaluates 1090 MHz reply downlink (27 dBW transponder ERP, -80 dBm interrogator MTL + 1090 MHz jamming)
        iffInRangeCount_ = 0;
        iffJammed_ = false;
        if (iffCapable_) {
            // Mode 5 MSK spread-spectrum processing gain (~12 dB) vs legacy unspread Mode S/3A
            const double iffAjLin = std::pow(10.0, (iffMode5_ ? 12.0 : 0.0) / 10.0);
            double ownJ01090 = 0.0;
            for (const Emission& e : scene.emissions()) {
                if (e.mode != EmitterMode::Jammer || !e.state || e.state == &own ||
                    e.deceptionMode != EaDeceptionModeV1::Noise) continue;
                const double ov1090 = ea::spectralOverlap(e.frequencyHz, e.bandwidthHz, kIffReplyHz, kIffBwHz);
                if (!(ov1090 > 0.0)) continue;
                const double txGain = e.gainTowardsDbi(own.positionEcef);
                const double lossDb = scene.pathLossDb(e.entity, kIffReplyHz);
                ownJ01090 += ea::receivedJammerDensityWHz(e.powerW, txGain, rxGainDbi_, lossDb, e.bandwidthHz, ov1090);
            }
            const double ownJamDeg1090Db = (ownJ01090 > 0.0)
                                               ? (10.0 * std::log10(1.0 + ownJ01090 / (n0 * iffAjLin)))
                                               : 0.0;
            iffJammed_ = ownJamDeg1090Db > 15.0;

            std::uint32_t ids[512];
            const sim::VehicleState* targets[512];
            const std::size_t n = scene.targets(ids, targets, 512);
            for (std::size_t k = 0; k < n; ++k) {
                if (!targets[k] || scene.role(ids[k]) != Role::Aircraft) continue;
                if (!scene.iffTransponderActive(ids[k])) continue;
                double los[3];
                const double r = geom::lineOfSight(own, targets[k]->positionEcef, los);
                if (!(r > 1.0) || scene.masked(ids[k], r)) continue;

                // Evaluate 1030 MHz noise jamming at the target transponder
                double tgtJ01030 = 0.0;
                for (const Emission& e : scene.emissions()) {
                    if (e.mode != EmitterMode::Jammer || !e.state || e.entity == ids[k] ||
                        e.deceptionMode != EaDeceptionModeV1::Noise) continue;
                    const double ov1030 = ea::spectralOverlap(e.frequencyHz, e.bandwidthHz, kIffInterrogateHz, kIffBwHz);
                    if (!(ov1030 > 0.0)) continue;
                    const double dx = targets[k]->positionEcef[0] - e.state->positionEcef[0];
                    const double dy = targets[k]->positionEcef[1] - e.state->positionEcef[1];
                    const double dz = targets[k]->positionEcef[2] - e.state->positionEcef[2];
                    const double jr = std::max(1.0, std::sqrt(dx * dx + dy * dy + dz * dz));
                    const double fsplDb = 20.0 * std::log10(4.0 * kPi * jr * kIffInterrogateHz / kLight);
                    const double txGain = e.gainTowardsDbi(targets[k]->positionEcef);
                    tgtJ01030 += ea::receivedJammerDensityWHz(e.powerW, txGain, 0.0, fsplDb, e.bandwidthHz, ov1030);
                }
                const double tgtJamDeg1030Db = (tgtJ01030 > 0.0)
                                                   ? (10.0 * std::log10(1.0 + tgtJ01030 / (n0 * iffAjLin)))
                                                   : 0.0;

                const double loss1030Db = scene.pathLossDb(ids[k], kIffInterrogateHz);
                // ICAO Annex 10 Vol. IV 3.1.2.10.1 nominal transponder Minimum Triggering Level (-74 dBm)
                if (iffLinkMarginDb(30.0, loss1030Db, 0.0, -74.0 + tgtJamDeg1030Db) < 0.0) continue;

                const double loss1090Db = scene.pathLossDb(ids[k], kIffReplyHz);
                if (iffLinkMarginDb(27.0, loss1090Db, rxGainDbi_, -80.0 + ownJamDeg1090Db) >= 0.0) {
                    ++iffInRangeCount_;
                }
            }
        }
    }

    void idle(double) override {
        active_ = false;
        jammed_ = false;
        iffJammed_ = false;
        iffInRangeCount_ = 0;
    }

    bool commLinkParams(CommLinkParams& out) const override {
        if (!active_) return false;
        out.bandMinHz = bandMinHz_;
        out.bandMaxHz = bandMaxHz_;
        out.txPowerW = txPowerW_;
        out.channelBwHz = channelBwHz_;
        out.processingGainDb = processingGainDb_;
        out.jamThresholdJToNDb = jamThresholdJToNDb_;
        out.rxGainDbi = rxGainDbi_;
        out.iffCapable = iffCapable_;
        out.iffMode5 = iffMode5_;
        return true;
    }

    bool isIffCapable() const override { return iffCapable_; }

    bool coversCommFrequency(double frequencyHz) const override {
        return std::isfinite(frequencyHz) && frequencyHz >= bandMinHz_ && frequencyHz <= bandMaxHz_;
    }

    void report(std::vector<PayloadValue>& out) const override {
        out.push_back({"active", active_ ? 1.0 : 0.0, ""});
        out.push_back({"link_available", (active_ && !jammed_) ? 1.0 : 0.0, ""});
        out.push_back({"jammed", jammed_ ? 1.0 : 0.0, ""});
        out.push_back({"jam_power", std::isfinite(jamPowerDbm_) ? jamPowerDbm_ : std::numeric_limits<double>::quiet_NaN(), "dBm"});
        out.push_back({"j_to_n", std::isfinite(effJToNDb_) ? effJToNDb_ : std::numeric_limits<double>::quiet_NaN(), "dB"});
        out.push_back({"tx_power", txPowerW_, "W"});
        out.push_back({"processing_gain", processingGainDb_, "dB"});
        out.push_back({"band_min", bandMinHz_, "Hz"});
        out.push_back({"band_max", bandMaxHz_, "Hz"});
        out.push_back({"iff_capable", iffCapable_ ? 1.0 : 0.0, ""});
        out.push_back({"iff_mode5", iffMode5_ ? 1.0 : 0.0, ""});
        out.push_back({"iff_jammed", iffJammed_ ? 1.0 : 0.0, ""});
        out.push_back({"iff_in_range", static_cast<double>(iffInRangeCount_), ""});
    }

private:
    double bandMinHz_ = 969.0e6;
    double bandMaxHz_ = 1206.0e6;
    double txPowerW_ = 200.0;
    double channelBwHz_ = 3.0e6;
    double processingGainDb_ = 17.0;
    double jamThresholdJToNDb_ = 25.0;
    double rxGainDbi_ = 0.0;
    bool iffCapable_ = false;
    bool iffMode5_ = false;

    bool active_ = false;
    bool jammed_ = false;
    bool iffJammed_ = false;
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
