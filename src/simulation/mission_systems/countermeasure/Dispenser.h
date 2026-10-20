#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

namespace fsim::ms::countermeasure {

enum class Kind { Chaff, Flare };

struct Point { double x = 0.0, y = 0.0, z = 0.0; };

struct Decoy {
    std::uint32_t id = 0, owner = 0;
    Kind kind = Kind::Chaff;
    double launchTimeS = 0.0, lifeS = 0.0; ///< Simulation launch timestamp and fixed lifetime, seconds.
    Point position, velocity;
};

/// Piecewise constant environment in the same Cartesian frame as the decoy.
/// down is normalized on acceptance; airVelocity is the modeled air's velocity.
struct Motion { Point down, airVelocity; };
struct Sample {
    Decoy state;
    double ageS = std::numeric_limits<double>::quiet_NaN();
};
/// Owned motion history for one decoy live at the beginning of a step. Expired
/// decoys retain their last partial trajectory after leaving the live list.
struct Trajectory {
    Sample before, activeEnd;
    double stepDurationS = 0.0, liveDurationS = 0.0;
    Motion motion;
    bool valid = false, expires = false;
};
/// Non-consuming analytic sample. The mathematical endpoint at liveDurationS
/// is allowed for geometry even when expires is true: visibility and fuze
/// callers MUST treat that expiry endpoint as exclusive. Invalid trajectories,
/// times and arithmetic leave out unchanged; no extrapolation is performed.
bool sampleAt(const Trajectory& trajectory, double offsetS, Sample& out) noexcept;

class Dispenser {
public:
    Dispenser(std::uint32_t owner, unsigned chaff, unsigned flare);
    /// Consume one cartridge on acceptance. IDs are allocated by the world, never by this module.
    /// All kinds share the assumed 0.35-second minimum interval; rejection has no side effects.
    bool dispense(Kind kind, double now, Point position, Point velocity, std::uint32_t id);
    /// Advance age and motion by simulation seconds. `down` is a nonzero direction, normalized here.
    /// Positions are metres and velocities metres/second in the caller's common Cartesian frame.
    /// This legacy wrapper supplies constant down and zero air velocity.
    /// Invalid steps leave the entire state unchanged. Invalid propagation
    /// removes its pose from the live list rather than publishing frozen motion.
    void step(double dt, Point down);
    /// Advance each decoy with its own supplied environment and return its owned
    /// before/active-end trajectory, including partial final lifetimes. Absolute
    /// step time belongs to the caller; samples retain compensated launch age.
    /// Invalid dt, missing callbacks or any invalid environment reject the whole
    /// step without mutation and return an empty result. Callbacks are evaluated
    /// before mutation; their exceptions propagate without changing state.
    /// Arithmetic failure returns a valid=false trajectory and removes that
    /// decoy's invalid pose from the live list. Inventory/cooldown are unchanged.
    std::vector<Trajectory> advance(double dt, std::function<Motion(const Decoy&)> environment);
    unsigned remaining(Kind kind) const;
    const std::vector<Decoy>& decoys() const;

private:
    std::uint32_t owner_;
    unsigned chaff_, flare_;
    double lastReleaseS_ = -1.0;
    std::vector<Decoy> decoys_;
    struct Age { double elapsedS = 0.0, correctionS = 0.0; };
    std::vector<Age> ages_;
};

} // namespace fsim::ms::countermeasure
