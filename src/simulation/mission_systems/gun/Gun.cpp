#include "simulation/mission_systems/gun/Gun.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string_view>

namespace fsim::ms::gun {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRho0 = 1.225; // ICAO ISA sea-level air density (kg/m^3)

class GunModel final : public Model {
public:
    explicit GunModel(Spec spec)
        : spec_(spec),
          dragK_(dragRetardationPerM(spec.calibreM, spec.projectileKg, spec.dragCd)),
          roundsRemaining_(spec.capacity) {}

    Reason begin(const PayloadTask& task, double now) override {
        if (task.kind != TaskKind::Fire) return Reason::InvalidParameter;
        if (task.rounds < 0) return Reason::InvalidParameter;
        if (std::isfinite(task.durationS) && !(task.durationS > 0.0)) return Reason::InvalidParameter;
        if (roundsRemaining_ <= 0) return Reason::Expended;

        firing_ = true;
        finishedReason_ = Reason::None;
        burstTarget_ = task.rounds; // 0 = continuous until canceled / durationS / expended
        burstFired_ = 0;
        burstStartS_ = now;
        lastStepS_ = now;
        endAtS_ = std::isfinite(task.durationS) ? (now + task.durationS) : std::numeric_limits<double>::infinity();
        // Start accumulator at 1.0 so the first shot fires promptly on the first active step
        // without phantom firing before step() executes.
        roundAccumulator_ = 1.0;
        return Reason::None;
    }

    void stop(double /*now*/) override {
        firing_ = false;
        burstTarget_ = 0;
        roundAccumulator_ = 0.0;
    }

    void idle(double /*now*/) override {
        firing_ = false;
        burstTarget_ = 0;
        roundAccumulator_ = 0.0;
    }

    Reason finished() const override { return finishedReason_; }

    double percent() const override {
        if (burstTarget_ > 0) {
            return std::clamp(100.0 * static_cast<double>(burstFired_) / static_cast<double>(burstTarget_), 0.0, 100.0);
        }
        if (std::isfinite(endAtS_) && endAtS_ > burstStartS_) {
            return std::clamp(100.0 * (lastStepS_ - burstStartS_) / (endAtS_ - burstStartS_), 0.0, 100.0);
        }
        return std::numeric_limits<double>::quiet_NaN();
    }

    void step(double now, const sim::VehicleState& /*own*/, const Scene& /*scene*/, Rng& /*rng*/) override {
        if (!firing_) {
            lastStepS_ = now;
            return;
        }
        const double dt = std::clamp(now - lastStepS_, 0.0, 0.5);
        lastStepS_ = now;

        const double roundsPerSec = std::max(spec_.rateOfFireRpm, 1.0) / 60.0;
        roundAccumulator_ += dt * roundsPerSec;
        int due = static_cast<int>(std::floor(roundAccumulator_));
        if (due > 0) {
            roundAccumulator_ -= static_cast<double>(due);
            if (burstTarget_ > 0) {
                due = std::min(due, burstTarget_ - burstFired_);
            }
            due = std::min(due, roundsRemaining_);
            if (due > 0) {
                roundsRemaining_ -= due;
                roundsFiredTotal_ += due;
                burstFired_ += due;
                pendingBurstRounds_ += due;
                pendingShedKg_ += static_cast<double>(due) * spec_.shedMassPerRoundKg;
            }
        }

        if (burstTarget_ > 0 && burstFired_ >= burstTarget_) {
            firing_ = false;
            finishedReason_ = Reason::GoalReached;
        } else if (roundsRemaining_ <= 0) {
            firing_ = false;
            finishedReason_ = Reason::Expended;
        } else if (now + 1e-9 >= endAtS_) {
            firing_ = false;
            finishedReason_ = Reason::GoalReached;
        }
    }

    bool takeBurst(GunBurst& out) override {
        if (pendingBurstRounds_ <= 0) return false;
        out.rounds = pendingBurstRounds_;
        out.muzzleMs = spec_.muzzleMs;
        out.projectileKg = spec_.projectileKg;
        out.calibreM = spec_.calibreM;
        out.dispersionRad = spec_.dispersionRad;
        out.explosiveKg = spec_.explosiveKg;
        out.dragK = dragK_;
        out.elevationRad = spec_.elevationRad;
        pendingBurstRounds_ = 0;
        return true;
    }

    double takeShedKg() override {
        const double kg = pendingShedKg_;
        pendingShedKg_ = 0.0;
        return kg;
    }

    void report(std::vector<PayloadValue>& out) const override {
        out.push_back({"rounds_remaining", static_cast<double>(roundsRemaining_), "rounds"});
        out.push_back({"rounds_fired", static_cast<double>(roundsFiredTotal_), "rounds"});
        out.push_back({"magazine_capacity", static_cast<double>(spec_.capacity), "rounds"});
        out.push_back({"firing", firing_ ? 1.0 : 0.0, "1"});
        out.push_back({"rate_of_fire_rpm", spec_.rateOfFireRpm, "rpm"});
        out.push_back({"muzzle_velocity_ms", spec_.muzzleMs, "m/s"});
        out.push_back({"calibre_mm", spec_.calibreM * 1000.0, "mm"});
        out.push_back({"ammunition_mass_kg", static_cast<double>(roundsRemaining_) * spec_.shedMassPerRoundKg, "kg"});
    }

private:
    Spec spec_{};
    double dragK_ = 0.0;
    int roundsRemaining_ = 0;
    int roundsFiredTotal_ = 0;
    bool firing_ = false;
    Reason finishedReason_ = Reason::None;
    int burstTarget_ = 0;
    int burstFired_ = 0;
    double burstStartS_ = 0.0;
    double lastStepS_ = 0.0;
    double endAtS_ = std::numeric_limits<double>::infinity();
    double roundAccumulator_ = 0.0;
    int pendingBurstRounds_ = 0;
    double pendingShedKg_ = 0.0;
};

} // namespace

double dragRetardationPerM(double calibreM, double projectileKg, double dragCd) noexcept {
    if (!(calibreM > 0.0) || !(projectileKg > 0.0) || !(dragCd > 0.0)) return 0.0;
    const double areaM2 = 0.25 * kPi * calibreM * calibreM;
    return 0.5 * kRho0 * dragCd * areaM2 / projectileKg;
}

Spec specOf(const Card& card, std::uint16_t count) noexcept {
    const int n = std::max<int>(1, static_cast<int>(count));
    const std::string_view id = card.id;
    Spec s;

    if (id == "m61a1") {
        s = {0.020, 6000.0, 1050.0, 0.101, 0.155, 0.011, 0.0020, 0.30, 0.0, 500};
    } else if (id == "m61a2") {
        s = {0.020, 6000.0, 1050.0, 0.101, 0.155, 0.011, 0.0020, 0.30, 0.0, 480};
    } else if (id == "gau-22a") {
        s = {0.025, 3300.0, 1040.0, 0.184, 0.285, 0.032, 0.0018, 0.29, 0.0, 182};
    } else if (id == "gau-8a") {
        s = {0.030, 3900.0, 1010.0, 0.360, 0.495, 0.058, 0.0020, 0.28, -0.035, 1174};
    } else if (id == "bk-27") {
        s = {0.027, 1700.0, 1025.0, 0.260, 0.515, 0.040, 0.0020, 0.29, 0.0, 150};
    } else if (id == "defa-554") {
        s = {0.030, 1800.0, 815.0, 0.244, 0.420, 0.048, 0.0025, 0.32, 0.0, 125};
    } else if (id == "giat-30m791") {
        s = {0.030, 2500.0, 1025.0, 0.275, 0.550, 0.050, 0.0020, 0.30, 0.0, 125};
    } else if (id == "gsh-23") {
        s = {0.023, 3400.0, 715.0, 0.175, 0.325, 0.019, 0.0025, 0.33, 0.0, 200};
    } else if (id == "gsh-30-1") {
        s = {0.030, 1500.0, 860.0, 0.390, 0.832, 0.048, 0.0022, 0.30, 0.0, 150};
    } else if (id == "gsh-30-2") {
        s = {0.030, 3000.0, 870.0, 0.390, 0.832, 0.048, 0.0022, 0.30, 0.0, 250};
    } else if (id == "m60d") {
        s = {0.00762, 550.0, 853.0, 0.0095, 0.025, 0.0, 0.0035, 0.34, 0.0, 500};
    } else {
        s = {0.020, 3000.0, 950.0, 0.100, 0.160, 0.010, 0.0025, 0.30, 0.0, 200};
    }

    // Scale rate of fire and capacity by installed gun mount count (e.g. 2x DEFA 554, 2x M60D).
    s.rateOfFireRpm *= static_cast<double>(n);
    s.capacity *= n;

    // Allow explicit card parameter overrides if present on future/updated cards.
    s.calibreM = card.value("calibre_m", s.calibreM);
    s.rateOfFireRpm = card.value("rate_of_fire_rpm", s.rateOfFireRpm);
    s.muzzleMs = card.value("muzzle_velocity_ms", s.muzzleMs);
    s.projectileKg = card.value("projectile_kg", s.projectileKg);
    s.shedMassPerRoundKg = card.value("shed_mass_kg", s.shedMassPerRoundKg);
    s.explosiveKg = card.value("explosive_kg", s.explosiveKg);
    s.dispersionRad = card.value("dispersion_rad", s.dispersionRad);
    s.dragCd = card.value("drag_cd", s.dragCd);
    s.elevationRad = card.value("elevation_rad", s.elevationRad);
    s.capacity = static_cast<int>(std::lround(card.value("capacity", static_cast<double>(s.capacity))));
    return s;
}

std::unique_ptr<Model> makeGun(const Card& card, std::uint16_t count) {
    if (card.family != Family::Gun) return nullptr;
    return std::make_unique<GunModel>(specOf(card, count));
}

} // namespace fsim::ms::gun
