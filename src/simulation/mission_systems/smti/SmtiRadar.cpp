#include "simulation/mission_systems/smti/SmtiRadar.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fsim::ms::smti {

namespace {

constexpr double kPi = geom::kPi;
constexpr double kDeg = kPi / 180.0;
constexpr double kLight = 299792458.0;

class SmtiRadar final : public Model {
public:
    explicit SmtiRadar(const Card& card) {
        frequencyHz_ = card.value("frequency_hz", 9.5e9);
        clutterSpreadHz_ = card.value("clutter_spread_hz", 160.0); // ~2.5 m/s MDV at X-band
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
    }

    Reason finished() const override { return finished_; }

    void step(double now, const sim::VehicleState& own, const Scene& scene, Rng&) override {
        if (!searching_) return;
        lastS_ = now;
        if (std::isfinite(durationS_) && now - startS_ + 1e-9 >= durationS_) {
            searching_ = false;
            finished_ = Reason::GoalReached;
            return;
        }
        contacts_.clear();
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

            Contact c;
            c.id = ++serial_;
            c.kind = ContactKind::Surface;
            c.firstTimeS = now;
            c.timeS = now;
            c.azimuthRad = az;
            c.elevationRad = el;
            c.bearingSigmaRad = 0.5 * kDeg;
            c.rangeM = rangeM;
            c.rangeRateMs = geom::dot3(tVel, los) - geom::dot3(ownVel, los);
            c.latitudeRad = st.latitudeRad;
            c.longitudeRad = st.longitudeRad;
            c.heightM = st.altitudeMslM;
            c.strengthDb = 18.0;
            c.truth = ids[k];
            contacts_.push_back(c);
        }
    }

    void idle(double now) override { stop(now); }
    void contacts(std::vector<Contact>& out) const override {
        for (const Contact& c : contacts_) out.push_back(c);
    }

    void report(std::vector<PayloadValue>& out) const override {
        out.push_back({"contacts", static_cast<double>(contacts_.size()), ""});
        out.push_back({"searching", searching_ ? 1.0 : 0.0, ""});
        out.push_back({"mdv", mdvMs_, "m/s"});
        out.push_back({"frequency", frequencyHz_, "Hz"});
    }

private:
    double frequencyHz_ = 9.5e9;
    double clutterSpreadHz_ = 160.0;
    double mdvMs_ = 2.5;
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
