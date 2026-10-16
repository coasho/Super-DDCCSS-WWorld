#include "simulation/mission_systems/esm/Receiver.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::ms::esm {

namespace {

constexpr double kDeg = geom::kPi / 180.0;

// What a card that says nothing of its receiver flies with - no maker publishes these for a type in service. One
// class for every country's, engineering assumptions each (a card's own figure, where one is found, replaces it):
constexpr double kSensitivityDbm = -60.0;   // the weakest pulse at its antenna it detects (a wide-open superheterodyne's class)
constexpr double kAntennaGainDbi = 0.0;     // a cavity-backed spiral's, as broad as a quadrant
constexpr double kBandMinHz = 2e9;          // 2 to 18 GHz: the band warning receivers are described as covering
constexpr double kBandMaxHz = 18e9;
constexpr double kBearingSigmaDeg = 10.0;   // amplitude comparison between four antennas
constexpr double kMeasureS = 1.0;           // a bearing is measured anew this often
constexpr double kMemoryS = 3.0;            // an emitter no longer heard is kept this long
constexpr std::size_t kContacts = 64;

class Receiver final : public Model {
public:
    explicit Receiver(const Card& card)
        : sensitivityDbm_(card.value("sensitivity_dbm", kSensitivityDbm)), antennaGainDbi_(card.value("antenna_gain_dbi", kAntennaGainDbi)),
          bandMinHz_(card.value("frequency_min_hz", kBandMinHz)), bandMaxHz_(card.value("frequency_max_hz", kBandMaxHz)),
          bearingSigmaRad_(card.value("bearing_sigma_deg", kBearingSigmaDeg) * kDeg), measureS_(card.value("measure_s", kMeasureS)),
          memoryS_(card.value("memory_s", kMemoryS)) {}

    void step(double now, const sim::VehicleState& own, const Scene& scene, Rng& rng) override {
        for (const Emission& e : scene.emissions()) {
            if (!e.state || e.state == &own) continue; // (its own carrier's transmitters: blanked)
            if (e.frequencyHz < bandMinHz_ || e.frequencyHz > bandMaxHz_) continue;
            double los[3];
            if (!(geom::lineOfSight(own, e.state->positionEcef, los) > 0.0)) continue;
            const double dbm = receivedDbm(e, own, scene.pathLossDb(e.entity, e.frequencyHz), antennaGainDbi_);
            if (!(dbm >= sensitivityDbm_)) continue;
            Held* h = held(e, now);
            if (!h) continue;
            if (now - h->measuredS >= measureS_) { // a new measurement: a new error
                h->measuredS = now;
                h->azimuthError = bearingSigmaRad_ * rng.normal();
                h->elevationError = bearingSigmaRad_ * rng.normal();
            }
            double body[3], azimuth = 0.0, elevation = 0.0;
            geom::toBody(own, los, body);
            geom::angles(body, azimuth, elevation);
            Contact& c = h->contact;
            c.timeS = now;
            c.azimuthRad = std::remainder(azimuth + h->azimuthError, 2.0 * geom::kPi);
            c.elevationRad = std::clamp(elevation + h->elevationError, -0.5 * geom::kPi, 0.5 * geom::kPi);
            c.bearingSigmaRad = bearingSigmaRad_;
            c.strengthDb = dbm;
            c.frequencyHz = e.frequencyHz;
            // What the emitter does, as its signal says: a beam that stays on the carrier is a lock (and, lighting it,
            // an illumination); one that passes as it scans, or a side lobe of one pointed elsewhere, is a search.
            const bool mainBeam = e.gainTowardsDbi(own.positionEcef) > 0.5 * (e.peakGainDbi + e.sidelobeDbi);
            switch (e.mode) {
            case EmitterMode::Lock:
            case EmitterMode::Illumination: c.mode = mainBeam ? e.mode : EmitterMode::Search; break;
            default: c.mode = e.mode; break;
            }
            c.warning = mainBeam && (c.mode == EmitterMode::Lock || c.mode == EmitterMode::Illumination || c.mode == EmitterMode::Seeker);
        }
        held_.erase(std::remove_if(held_.begin(), held_.end(), [&](const Held& h) { return now - h.contact.timeS > memoryS_; }), held_.end());
    }

    void idle(double) override { held_.clear(); }

    void contacts(std::vector<Contact>& out) const override {
        for (const Held& h : held_) out.push_back(h.contact);
    }

    void report(std::vector<PayloadValue>& out) const override {
        double warnings = 0.0;
        for (const Held& h : held_) warnings += h.contact.warning ? 1.0 : 0.0;
        out.push_back({"contacts", static_cast<double>(held_.size()), ""});
        out.push_back({"warnings", warnings, ""});
        out.push_back({"sensitivity", sensitivityDbm_, "dBm"});
    }

private:
    struct Held {
        Contact contact;
        std::uint32_t entity = 0;
        std::uint16_t payload = 0; ///< the emitter's own, on its carrier
        double measuredS = -std::numeric_limits<double>::infinity();
        double azimuthError = 0.0, elevationError = 0.0;
    };

    Held* held(const Emission& e, double now) {
        for (Held& h : held_)
            if (h.entity == e.entity && h.payload == e.payload) return &h;
        if (held_.size() >= kContacts) return nullptr;
        Held h;
        h.entity = e.entity, h.payload = e.payload;
        h.contact.id = ++serial_;
        h.contact.kind = ContactKind::Emitter;
        h.contact.firstTimeS = now;
        h.contact.truth = e.entity;
        held_.push_back(h);
        return &held_.back();
    }

    double sensitivityDbm_, antennaGainDbi_, bandMinHz_, bandMaxHz_, bearingSigmaRad_, measureS_, memoryS_;
    std::vector<Held> held_;
    std::uint64_t serial_ = 0;
};

} // namespace

double receivedDbm(const Emission& emission, const sim::VehicleState& own, double pathLossDb, double antennaGainDbi) noexcept {
    const double watts = emission.peakPowerW > 0.0 ? emission.peakPowerW : emission.powerW;
    return 10.0 * std::log10(std::max(watts, 1e-12) * 1000.0) + emission.gainTowardsDbi(own.positionEcef) - pathLossDb + antennaGainDbi;
}

std::unique_ptr<Model> makeReceiver(const Card& card) { return std::make_unique<Receiver>(card); }

} // namespace fsim::ms::esm
