#include "simulation/mission_systems/countermeasure/Dispenser.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

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

bool valid(Motion& motion) {
    return finite(motion.airVelocity) && normalize(motion.down);
}

double dragOf(Kind kind) { return kind == Kind::Chaff ? kChaffDragPerS : kFlareDragPerS; }

bool component(double position, double velocity, double down, double airVelocity, double dt, double drag,
               long double attenuation, long double integratedVelocity, double& nextPosition, double& nextVelocity) {
    // dv/dt = gravity * down - drag * (v - airVelocity), with the caller's
    // constant local environment. Long intermediates avoid premature overflow.
    const long double acceleration = static_cast<long double>(kGravityMs2) * down;
    const long double relative = static_cast<long double>(velocity) - airVelocity;
    const long double p = static_cast<long double>(position) +
                          static_cast<long double>(airVelocity) * dt + relative * integratedVelocity +
                          acceleration / drag * (static_cast<long double>(dt) - integratedVelocity);
    const long double v = airVelocity + relative * attenuation + acceleration * integratedVelocity;
    const long double limit = std::numeric_limits<double>::max();
    if (!std::isfinite(p) || !std::isfinite(v) || std::fabs(p) > limit || std::fabs(v) > limit) return false;
    nextPosition = static_cast<double>(p);
    nextVelocity = static_cast<double>(v);
    return true;
}

bool propagate(const Decoy& before, const Motion& motion, double dt, Decoy& after) {
    const double drag = dragOf(before.kind);
    const long double attenuation = std::exp(-static_cast<long double>(drag) * dt);
    const long double integratedVelocity = -std::expm1(-static_cast<long double>(drag) * dt) / drag;
    Point position, velocity;
    if (!component(before.position.x, before.velocity.x, motion.down.x, motion.airVelocity.x, dt, drag,
                   attenuation, integratedVelocity, position.x, velocity.x)
        || !component(before.position.y, before.velocity.y, motion.down.y, motion.airVelocity.y, dt, drag,
                      attenuation, integratedVelocity, position.y, velocity.y)
        || !component(before.position.z, before.velocity.z, motion.down.z, motion.airVelocity.z, dt, drag,
                      attenuation, integratedVelocity, position.z, velocity.z)) return false;
    after = before;
    after.position = position;
    after.velocity = velocity;
    return true;
}
} // namespace

bool sampleAt(const Trajectory& trajectory, double offsetS, Sample& out) noexcept {
    const auto& before = trajectory.before;
    const auto& end = trajectory.activeEnd;
    Motion motion = trajectory.motion;
    if (!trajectory.valid || !std::isfinite(offsetS) || offsetS < 0
        || !std::isfinite(trajectory.stepDurationS) || trajectory.stepDurationS <= 0
        || !std::isfinite(trajectory.liveDurationS) || trajectory.liveDurationS < 0
        || trajectory.liveDurationS > trajectory.stepDurationS || offsetS > trajectory.liveDurationS
        || !std::isfinite(before.ageS) || before.ageS < 0 || !std::isfinite(end.ageS) || end.ageS < before.ageS
        || (before.state.kind != Kind::Chaff && before.state.kind != Kind::Flare)
        || !std::isfinite(before.state.lifeS) || before.state.lifeS <= 0
        || before.ageS >= before.state.lifeS || end.ageS > before.state.lifeS
        || !finite(before.state.position) || !finite(before.state.velocity)
        || !finite(end.state.position) || !finite(end.state.velocity) || !valid(motion)) return false;
    if (offsetS == 0) { out = before; return true; }
    if (offsetS == trajectory.liveDurationS) { out = end; return true; }
    Sample sample;
    sample.ageS = before.ageS + offsetS;
    if (!std::isfinite(sample.ageS) || sample.ageS > before.state.lifeS
        || !propagate(before.state, motion, offsetS, sample.state)) return false;
    out = sample;
    return true;
}

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
    (void)advance(dt, [down](const Decoy&) { return Motion{down, {}}; });
}

std::vector<Trajectory> Dispenser::advance(double dt, std::function<Motion(const Decoy&)> environment) {
    if (!std::isfinite(dt) || dt <= 0 || !environment) return {};
    std::vector<Motion> motions;
    motions.reserve(decoys_.size());
    for (const auto& decoy : decoys_) {
        Motion motion = environment(decoy);
        if (!valid(motion)) return {};
        motions.push_back(motion);
    }
    std::vector<Trajectory> trajectories;
    std::vector<Decoy> survivors;
    std::vector<Age> survivorAges;
    trajectories.reserve(decoys_.size());
    survivors.reserve(decoys_.size());
    survivorAges.reserve(decoys_.size());
    for (std::size_t index = 0; index < decoys_.size(); ++index) {
        const auto& decoy = decoys_[index];
        Age age = ages_[index];
        Trajectory path;
        path.before = {decoy, age.elapsedS};
        path.stepDurationS = dt;
        path.motion = motions[index];
        const double remaining = std::max(0.0, decoy.lifeS - age.elapsedS);
        path.expires = dt >= remaining;
        if (!path.expires) {
            // Compensated summation avoids an extra lifetime tick after many
            // fractional fixed steps; large steps are clipped before addition.
            const double correctedDt = dt - age.correctionS;
            const double elapsed = age.elapsedS + correctedDt;
            age.correctionS = (elapsed - age.elapsedS) - correctedDt;
            age.elapsedS = elapsed;
            path.expires = elapsed >= decoy.lifeS;
        }
        path.liveDurationS = path.expires ? std::min(dt, remaining) : dt;
        path.activeEnd.ageS = path.expires ? decoy.lifeS : age.elapsedS;
        path.valid = propagate(decoy, path.motion, path.liveDurationS, path.activeEnd.state);
        if (path.valid && !path.expires) {
            survivors.push_back(path.activeEnd.state);
            survivorAges.push_back(age);
        } else if (!path.valid) {
            // No stale pose may stand in for successfully advanced motion.
            const double nan = std::numeric_limits<double>::quiet_NaN();
            path.activeEnd.ageS = nan;
            path.activeEnd.state = decoy;
            path.activeEnd.state.position = path.activeEnd.state.velocity = {nan, nan, nan};
        }
        trajectories.push_back(std::move(path));
    }
    decoys_.swap(survivors);
    ages_.swap(survivorAges);
    return trajectories;
}

unsigned Dispenser::remaining(Kind kind) const {
    return kind == Kind::Chaff ? chaff_ : kind == Kind::Flare ? flare_ : 0;
}

const std::vector<Decoy>& Dispenser::decoys() const { return decoys_; }
} // namespace fsim::ms::countermeasure
