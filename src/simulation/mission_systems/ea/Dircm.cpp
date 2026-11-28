#include "simulation/mission_systems/ea/Dircm.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::ms::ea {

namespace {

constexpr double kPi = geom::kPi;
constexpr double kDeg = kPi / 180.0;

bool isDircmCard(const Card& card) noexcept {
    return card.id == "an-aaq-24" || card.value("is_dircm", 0.0) > 0.5;
}

class Dircm final : public Model {
public:
    Dircm(const Card& card, std::uint16_t count)
        : turrets_(std::max<std::uint16_t>(1, count)),
          laserPowerW_(card.value("laser_power_w", 8.0)),
          lowUm_(card.value("wavelength_low_um", 3.7)),
          highUm_(card.value("wavelength_high_um", 4.6)),
          halfDivRad_(0.5 * card.value("beam_divergence_mrad", 2.5) * 1e-3),
          modulationHz_(card.value("modulation_hz", 115.0)),
          slewRateRadS_(card.value("slew_rate_deg_s", 120.0) * kDeg),
          lockTimeS_(card.value("lock_time_s", 0.35)),
          maxRangeM_(card.value("max_range_m", 12000.0)),
          elMinRad_(card.value("for_el_min_deg", -85.0) * kDeg),
          elMaxRad_(card.value("for_el_max_deg", turrets_ >= 2 ? 75.0 : 35.0) * kDeg),
          fineTrackFovRad_(card.value("fine_track_fov_deg", 3.0) * kDeg) {}

    Reason begin(const PayloadTask& task, double now) override {
        if (task.kind != TaskKind::Jam) return Reason::InvalidParameter;
        if (std::isfinite(task.durationS) && !(task.durationS > 0.0)) return Reason::InvalidParameter;
        // Reject explicit RF frequency commands on an optical DIRCM turret.
        if (std::isfinite(task.frequencyHz)) return Reason::InvalidParameter;
        taskActive_ = true;
        finished_ = Reason::None;
        startS_ = now;
        lastS_ = now;
        durationS_ = std::isfinite(task.durationS) ? task.durationS : std::numeric_limits<double>::quiet_NaN();
        followContact_ = task.contact;
        contactAcquired_ = (task.contact == 0);
        if (task.contact == 0) {
            cmdAzRad_ = std::isfinite(task.azimuthCentreRad) ? std::remainder(task.azimuthCentreRad, 2.0 * kPi) : 0.0;
            cmdElRad_ = std::isfinite(task.elevationCentreRad) ? std::clamp(task.elevationCentreRad, elMinRad_, elMaxRad_) : 0.0;
        }
        return Reason::None;
    }

    void stop(double) override {
        taskActive_ = false;
        followContact_ = 0;
        contactAcquired_ = false;
        finished_ = Reason::None;
    }

    Reason finished() const override { return finished_; }

    double percent() const override {
        if (!taskActive_ || !std::isfinite(durationS_) || durationS_ <= 0.0) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return std::clamp(100.0 * (lastS_ - startS_) / durationS_, 0.0, 100.0);
    }

    std::uint64_t follows() const override { return taskActive_ ? followContact_ : 0; }

    void followed(const Contact* held) override {
        if (!taskActive_ || followContact_ == 0) return;
        if (!held) {
            taskActive_ = false;
            contactAcquired_ = false;
            finished_ = Reason::ContactLost;
            return;
        }
        if (held->kind != ContactKind::Infrared) {
            taskActive_ = false;
            contactAcquired_ = false;
            finished_ = Reason::InvalidParameter;
            return;
        }
        cmdAzRad_ = std::remainder(held->azimuthRad, 2.0 * kPi);
        cmdElRad_ = std::clamp(held->elevationRad, elMinRad_, elMaxRad_);
        cuedTarget_ = held->truth;
        contactAcquired_ = true;
    }

    void step(double now, const sim::VehicleState& own, const Scene& scene, Rng&) override {
        const double dt = (hasStepped_ && now > lastS_) ? std::min(now - lastS_, 1.0) : 0.0;
        hasStepped_ = true;
        lastS_ = now;
        lastOwn_ = own;
        haveOwn_ = true;

        if (taskActive_ && std::isfinite(durationS_) && now - startS_ + 1e-9 >= durationS_) {
            taskActive_ = false;
            finished_ = Reason::GoalReached;
        }

        bool haveGoal = false;
        double goalAz = turretAzRad_;
        double goalEl = turretElRad_;
        std::uint32_t goalTarget = 0;

        if (taskActive_ && contactAcquired_) {
            haveGoal = true;
            goalAz = cmdAzRad_;
            goalEl = cmdElRad_;
            goalTarget = cuedTarget_;
        } else {
            // Autonomous LAIRCM self-defense: fine-track closest closing or burning
            // incoming missile inside turret field of regard and optical line of sight.
            constexpr std::size_t kMax = 512;
            std::uint32_t ids[kMax];
            const sim::VehicleState* states[kMax];
            const std::size_t n = scene.targets(ids, states, kMax);
            double bestRange = maxRangeM_;
            double vOwn[3];
            geom::velocityEcef(own, vOwn);
            for (std::size_t k = 0; k < n; ++k) {
                const std::uint32_t id = ids[k];
                if (id >= kDecoyIds || scene.role(id) != Role::Weapon) continue;
                const sim::VehicleState& t = *states[k];
                if (t.crashed || t.diverged) continue;
                double los[3];
                const double range = geom::lineOfSight(own, t.positionEcef, los);
                if (!(range > 20.0) || range > bestRange) continue;
                double vTgt[3];
                geom::velocityEcef(t, vTgt);
                const double closingMs = -((vTgt[0] - vOwn[0]) * los[0] +
                                           (vTgt[1] - vOwn[1]) * los[1] +
                                           (vTgt[2] - vOwn[2]) * los[2]);
                const bool burning = scene.weaponBurning(id);
                if (!burning && closingMs < 35.0) continue;
                if (scene.masked(id, range)) continue;
                double body[3];
                geom::toBody(own, los, body);
                double az = 0.0, el = 0.0;
                geom::angles(body, az, el);
                if (el < elMinRad_ || el > elMaxRad_) continue;
                bestRange = range;
                goalAz = az;
                goalEl = el;
                goalTarget = id;
                haveGoal = true;
            }
        }

        if (!haveGoal) {
            lasing_ = false;
            trackedTarget_ = 0;
            lockAccumS_ = 0.0;
            return;
        }

        if (goalTarget != trackedTarget_ && goalTarget != 0) {
            // New threat handover requires fresh fine-track acquisition lock.
            lockAccumS_ = 0.0;
            trackedTarget_ = goalTarget;
        } else if (goalTarget == 0) {
            trackedTarget_ = 0;
        }

        // Finite turret servo slew toward commanded bearing.
        const double maxStep = (dt > 0.0) ? (slewRateRadS_ * dt) : 0.0;
        const double dAz = std::remainder(goalAz - turretAzRad_, 2.0 * kPi);
        const double dEl = goalEl - turretElRad_;
        const double err = std::hypot(dAz, dEl);
        if (err <= maxStep || err <= 1e-9) {
            turretAzRad_ = std::remainder(goalAz, 2.0 * kPi);
            turretElRad_ = std::clamp(goalEl, elMinRad_, elMaxRad_);
        } else if (maxStep > 0.0) {
            const double scale = maxStep / err;
            turretAzRad_ = std::remainder(turretAzRad_ + dAz * scale, 2.0 * kPi);
            turretElRad_ = std::clamp(turretElRad_ + dEl * scale, elMinRad_, elMaxRad_);
        }

        const double postErr = std::hypot(std::remainder(goalAz - turretAzRad_, 2.0 * kPi), goalEl - turretElRad_);
        if (postErr <= fineTrackFovRad_) {
            lockAccumS_ = std::min(lockTimeS_ + 1.0, lockAccumS_ + dt);
            lasing_ = (lockAccumS_ + 1e-9 >= lockTimeS_);
        } else {
            lockAccumS_ = 0.0;
            lasing_ = false;
        }

        // Modulated MWIR jam-code waveform (average power = laserPowerW_).
        const double mod = 0.25 + 0.75 * (1.0 + std::sin(2.0 * kPi * modulationHz_ * now));
        instantaneousPowerW_ = lasing_ ? (laserPowerW_ * mod) : 0.0;
    }

    void idle(double now) override {
        stop(now);
        lasing_ = false;
        trackedTarget_ = 0;
        lockAccumS_ = 0.0;
        instantaneousPowerW_ = 0.0;
        haveOwn_ = false;
    }

    bool dircmBeam(DircmBeam& out) const override {
        if (!lasing_ || !haveOwn_ || !(instantaneousPowerW_ > 0.0)) return false;
        std::copy_n(lastOwn_.positionEcef, 3, out.originEcef);
        geom::velocityEcef(lastOwn_, out.velocityEcef);
        const double cb = std::cos(turretElRad_);
        const double bodyDir[3] = {cb * std::cos(turretAzRad_), cb * std::sin(turretAzRad_), -std::sin(turretElRad_)};
        geom::toEcef(lastOwn_, bodyDir, out.aimDirEcef);
        const double n = geom::norm3(out.aimDirEcef);
        if (!(n > 1e-9)) return false;
        for (double& c : out.aimDirEcef) c /= n;
        out.powerW = instantaneousPowerW_;
        out.halfDivergenceRad = halfDivRad_;
        out.lowUm = lowUm_;
        out.highUm = highUm_;
        out.maxRangeM = maxRangeM_;
        out.target = trackedTarget_;
        return true;
    }

    void report(std::vector<PayloadValue>& out) const override {
        out.push_back({"active", haveOwn_ ? 1.0 : 0.0, ""});
        out.push_back({"lasing", lasing_ ? 1.0 : 0.0, ""});
        out.push_back({"power", lasing_ ? instantaneousPowerW_ : 0.0, "W"});
        out.push_back({"wavelength_low", lowUm_, "um"});
        out.push_back({"wavelength_high", highUm_, "um"});
        out.push_back({"beam_divergence", 2.0 * halfDivRad_ * 1e3, "mrad"});
        out.push_back({"azimuth", turretAzRad_ / kDeg, "deg"});
        out.push_back({"elevation", turretElRad_ / kDeg, "deg"});
        out.push_back({"tracked_threat", static_cast<double>(trackedTarget_), ""});
    }

private:
    std::uint16_t turrets_ = 1;
    double laserPowerW_ = 8.0;
    double lowUm_ = 3.7;
    double highUm_ = 4.6;
    double halfDivRad_ = 0.00125;
    double modulationHz_ = 115.0;
    double slewRateRadS_ = 120.0 * kDeg;
    double lockTimeS_ = 0.35;
    double maxRangeM_ = 12000.0;
    double elMinRad_ = -85.0 * kDeg;
    double elMaxRad_ = 35.0 * kDeg;
    double fineTrackFovRad_ = 3.0 * kDeg;

    bool taskActive_ = false;
    bool contactAcquired_ = false;
    bool hasStepped_ = false;
    bool haveOwn_ = false;
    bool lasing_ = false;
    Reason finished_ = Reason::None;
    double startS_ = 0.0;
    double lastS_ = 0.0;
    double durationS_ = std::numeric_limits<double>::quiet_NaN();
    std::uint64_t followContact_ = 0;
    std::uint32_t cuedTarget_ = 0;
    std::uint32_t trackedTarget_ = 0;
    double cmdAzRad_ = 0.0;
    double cmdElRad_ = 0.0;
    double turretAzRad_ = 0.0;
    double turretElRad_ = 0.0;
    double lockAccumS_ = 0.0;
    double instantaneousPowerW_ = 0.0;
    sim::VehicleState lastOwn_{};
};

} // namespace

std::unique_ptr<Model> makeDircm(const Card& card, std::uint16_t count) {
    if (!isDircmCard(card)) return nullptr;
    return std::make_unique<Dircm>(card, count);
}

} // namespace fsim::ms::ea
