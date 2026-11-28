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
        out.maxFlightTimeS = spec_.maxFlightTimeS;
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

double g7DragCoefficient(double mach) noexcept {
    // Published BRL / McCoy G7 standard projectile drag function Cd(M)
    // (R. L. McCoy, "Modern Exterior Ballistics", Schiffer 1999; NATO STANAG 4355).
    static constexpr double kMach[] = {
        0.00, 0.70, 0.80, 0.85, 0.90, 0.925, 0.95, 0.975, 1.00, 1.025,
        1.05, 1.10, 1.20, 1.40, 1.60, 1.80,  2.00, 2.50,  3.00, 4.00, 5.00};
    static constexpr double kCd[] = {
        0.1198, 0.1197, 0.1235, 0.1306, 0.1464, 0.1660, 0.2034, 0.2909, 0.3803, 0.4015,
        0.4043, 0.4014, 0.3802, 0.3485, 0.3198, 0.2980, 0.2809, 0.2469, 0.2225, 0.1935, 0.1750};
    constexpr std::size_t kN = sizeof(kMach) / sizeof(kMach[0]);
    if (!std::isfinite(mach) || mach <= kMach[0]) return kCd[0];
    if (mach >= kMach[kN - 1]) return kCd[kN - 1];
    for (std::size_t i = 1; i < kN; ++i) {
        if (mach <= kMach[i]) {
            const double t = (mach - kMach[i - 1]) / (kMach[i] - kMach[i - 1]);
            return kCd[i - 1] + t * (kCd[i] - kCd[i - 1]);
        }
    }
    return kCd[kN - 1];
}

double dragRetardationPerM(double calibreM, double projectileKg, double dragCd) noexcept {
    if (!(calibreM > 0.0) || !(projectileKg > 0.0) || !(dragCd > 0.0)) return 0.0;
    const double areaM2 = 0.25 * kPi * calibreM * calibreM;
    return 0.5 * kRho0 * dragCd * areaM2 / projectileKg;
}

Spec specOf(const Card& card, std::uint16_t count) noexcept {
    const int n = std::max<int>(1, static_cast<int>(count));
    // Generic 20 mm single-mount engineering fallback only when a synthetic card omits parameters;
    // all maintained gun cards carry calibrated single-mount parameters in assets/payloads/*.json.
    Spec s{0.020, 3000.0, 950.0, 0.100, 0.160, 0.010, 0.0025, 0.30, 0.0, 200, 6.0};

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
    s.maxFlightTimeS = card.value("self_destruct_s", s.maxFlightTimeS);

    // Scale per-mount cyclic rate and magazine capacity by installed gun mount count (e.g. 2x DEFA 554, 2x M60D).
    s.rateOfFireRpm *= static_cast<double>(n);
    s.capacity *= n;
    return s;
}

std::unique_ptr<Model> makeGun(const Card& card, std::uint16_t count) {
    if (card.family != Family::Gun) return nullptr;
    return std::make_unique<GunModel>(specOf(card, count));
}

} // namespace fsim::ms::gun
