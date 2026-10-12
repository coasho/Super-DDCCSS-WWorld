#pragma once

#include <cstdint>
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

class Dispenser {
public:
    Dispenser(std::uint32_t owner, unsigned chaff, unsigned flare);
    /// Consume one cartridge on acceptance. IDs are allocated by the world, never by this module.
    /// All kinds share the assumed 0.35-second minimum interval; rejection has no side effects.
    bool dispense(Kind kind, double now, Point position, Point velocity, std::uint32_t id);
    /// Advance age and motion by simulation seconds. `down` is a nonzero direction, normalized here.
    /// Positions are metres and velocities metres/second in the caller's common Cartesian frame.
    /// Motion assumes still air: no wind velocity is supplied by this interface.
    /// Invalid steps leave the entire state unchanged. Arithmetic overflow preserves prior motion.
    void step(double dt, Point down);
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
