#include "simulation/mission_systems/countermeasure/Dispenser.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::ms::countermeasure {
namespace {
// Basis: assumed engineering constants, not calibration of any real dispenser or cartridge.
constexpr double kIntervalS = 0.35;
constexpr double kGravityMs2 = 9.80665;
constexpr double kChaffLifeS = 6.0;
constexpr double kFlareLifeS = 4.0;
constexpr double kChaffDragPerS = 4.0;
constexpr double kFlareDragPerS = 0.4;

bool finite(Point p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

bool normalize(Point& p) {
    if (!finite(p)) return false;
    const double scale = std::max({std::fabs(p.x), std::fabs(p.y), std::fabs(p.z)});
    if (scale == 0.0) return false;
    p.x /= scale;
    p.y /= scale;
    p.z /= scale;
    const double length = std::hypot(p.x, p.y, p.z);
    p.x /= length;
    p.y /= length;
    p.z /= length;
    return true;
}

bool component(double position, double velocity, double down, double dt, double drag,
               double attenuation, double integratedVelocity, double& nextPosition, double& nextVelocity) {
    // Integrate dv/dt = gravity * down - drag * v analytically. Long intermediates prevent premature overflow.
    const long double acceleration = static_cast<long double>(kGravityMs2) * down;
    const long double p = static_cast<long double>(position) +
                          static_cast<long double>(velocity) * integratedVelocity +
                          acceleration / drag * (static_cast<long double>(dt) - integratedVelocity);
    const long double v = static_cast<long double>(velocity) * attenuation + acceleration * integratedVelocity;
    const long double limit = std::numeric_limits<double>::max();
    if (!std::isfinite(p) || !std::isfinite(v) || std::fabs(p) > limit || std::fabs(v) > limit) return false;
    nextPosition = static_cast<double>(p);
    nextVelocity = static_cast<double>(v);
    return true;
}
} // namespace

Dispenser::Dispenser(std::uint32_t owner, unsigned chaff, unsigned flare) : owner_(owner), chaff_(chaff), flare_(flare) {}

bool Dispenser::dispense(Kind kind, double now, Point position, Point velocity, std::uint32_t id) {
    if ((kind != Kind::Chaff && kind != Kind::Flare) || !std::isfinite(now) || now < 0.0 ||
        !finite(position) || !finite(velocity) || id == 0 || remaining(kind) == 0) return false;
    if (lastReleaseS_ >= 0.0 && (now < lastReleaseS_ || now < lastReleaseS_ + kIntervalS)) return false;
    if (std::any_of(decoys_.begin(), decoys_.end(), [id](const Decoy& decoy) { return decoy.id == id; })) return false;

    ages_.push_back({});
    try {
        decoys_.push_back({id, owner_, kind, now, kind == Kind::Chaff ? kChaffLifeS : kFlareLifeS, position, velocity});
    } catch (...) {
        ages_.pop_back();
        throw;
    }
    --(kind == Kind::Chaff ? chaff_ : flare_);
    lastReleaseS_ = now;
    return true;
}

void Dispenser::step(double dt, Point down) {
    if (!std::isfinite(dt) || dt <= 0.0 || !normalize(down)) return;
    std::size_t surviving = 0;
    for (std::size_t index = 0; index < decoys_.size(); ++index) {
        Decoy decoy = decoys_[index];
        Age age = ages_[index];
        // Check remaining time before addition or motion, including arbitrarily large finite steps.
        if (dt >= decoy.lifeS - age.elapsedS) continue;
        // Compensated summation avoids one extra lifetime tick after many fractional fixed steps.
        const double correctedDt = dt - age.correctionS;
        const double elapsed = age.elapsedS + correctedDt;
        age.correctionS = (elapsed - age.elapsedS) - correctedDt;
        age.elapsedS = elapsed;
        if (age.elapsedS >= decoy.lifeS) continue;
        const double drag = decoy.kind == Kind::Chaff ? kChaffDragPerS : kFlareDragPerS;
        const double attenuation = std::exp(-drag * dt);
        const double integratedVelocity = -std::expm1(-drag * dt) / drag;
        Point position, velocity;
        if (component(decoy.position.x, decoy.velocity.x, down.x, dt, drag, attenuation, integratedVelocity, position.x, velocity.x) &&
            component(decoy.position.y, decoy.velocity.y, down.y, dt, drag, attenuation, integratedVelocity, position.y, velocity.y) &&
            component(decoy.position.z, decoy.velocity.z, down.z, dt, drag, attenuation, integratedVelocity, position.z, velocity.z)) {
            decoy.position = position;
            decoy.velocity = velocity;
        }
        decoys_[surviving] = decoy;
        ages_[surviving] = age;
        ++surviving;
    }
    decoys_.resize(surviving);
    ages_.resize(surviving);
}

unsigned Dispenser::remaining(Kind kind) const {
    return kind == Kind::Chaff ? chaff_ : kind == Kind::Flare ? flare_ : 0;
}

const std::vector<Decoy>& Dispenser::decoys() const { return decoys_; }
} // namespace fsim::ms::countermeasure
