#include "simulation/mission_systems/ea/Jammer.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::ms::ea {

namespace {

constexpr double kPi = geom::kPi;
constexpr double kDeg = kPi / 180.0;

// Engineering class defaults (basis: assumed) when a payload card omits
// unpublished jammer transmitter parameters:
// - Tactical escort/standoff pods (e.g. AN/ALQ-99, Compass Call): higher ERP,
//   wider band, directional steerable beam.
// - Self-protection jammers (e.g. AN/ALQ-135, AN/ALQ-165): moderate power,
//   wide sector/quadrant coverage.
struct ClassDefaults {
    bool isRf = true;
    double powerW = 400.0;
    double peakGainDbi = 10.0;
    double sidelobeDbi = -6.0;
    double bandMinHz = 2.0e9;
    double bandMaxHz = 18.0e9;
    double defaultAzHalfRad = 30.0 * kDeg;
    double defaultElHalfRad = 20.0 * kDeg;
    double spotBandwidthHz = 50.0e6;
    double minBandwidthHz = 1.0e6;
};

ClassDefaults defaultsFor(const Card& card) {
    ClassDefaults d;
    if (card.id == "an-aaq-24") {
        // Directional Infrared Countermeasures (laser DIRCM) is optical/IR, not RF.
        d.isRf = false;
        return d;
    }
    if (card.id == "an-alq-99") {
        d.powerW = 1500.0;
        d.peakGainDbi = 14.0;
        d.sidelobeDbi = -8.0;
        d.bandMinHz = 0.064e9;
        d.bandMaxHz = 18.0e9;
        d.defaultAzHalfRad = 20.0 * kDeg;
        d.defaultElHalfRad = 15.0 * kDeg;
        d.spotBandwidthHz = 40.0e6;
    } else if (card.id == "compass-call") {
        d.powerW = 2000.0;
        d.peakGainDbi = 12.0;
        d.sidelobeDbi = -8.0;
        d.bandMinHz = 0.02e9;
        d.bandMaxHz = 6.0e9;
        d.defaultAzHalfRad = 35.0 * kDeg;
        d.defaultElHalfRad = 20.0 * kDeg;
        d.spotBandwidthHz = 20.0e6;
    } else if (card.id == "an-alq-172" || card.id == "an-alq-155" || card.id == "an-alt-16" ||
               card.id == "an-alt-28" || card.id == "an-alt-32") {
        d.powerW = 800.0;
        d.peakGainDbi = 11.0;
        d.bandMinHz = 0.5e9;
        d.bandMaxHz = 18.0e9;
    } else if (card.id == "l402-himalayas" || card.id == "an-alq-135" || card.id == "an-alq-165") {
        d.powerW = 500.0;
        d.peakGainDbi = 10.0;
        d.bandMinHz = 2.0e9;
        d.bandMaxHz = 18.0e9;
    }
    return d;
}

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
          minBandwidthHz_(card.value("min_bandwidth_hz", def.minBandwidthHz)) {}

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

std::unique_ptr<Model> makeJammer(const Card& card, std::uint16_t count) {
    const ClassDefaults def = defaultsFor(card);
    if (!def.isRf) return nullptr;
    return std::make_unique<Jammer>(card, def, count);
}

} // namespace fsim::ms::ea
