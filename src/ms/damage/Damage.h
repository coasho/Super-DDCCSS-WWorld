#pragma once

// A warhead's damage to an aircraft (ADR-30, MS-7g's damage part, docs/mission-systems.md G1 - G4): the physical
// chain from a detonation to what it breaks - the warhead's fragments (Gurney's velocity, the spray's zones, slowed
// by the air), each one's flight across the target in its attitude, its penetration of the skin (THOR) and what it
// strikes behind it, and the blast (free-air Kingery-Bulmash, scaled for altitude) on each skin panel. Pure: the
// world (session/World) owns the state and draws the chances from the vehicle's own damage stream.
//
// Frames: the target's body axes from its model's origin (x forward, y right, z down; m) as its damage data
// (aircraft/<type>/<type>.damage.json, tools/hangar/hangar/damage.py) and its manifest's components are; ECEF for
// the burst and the target's pose.

#include "fsim/Rng.h"
#include "ms/Cards.h"

#include <array>
#include <cmath>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace fsim::ms::damage {

struct V3 {
    double x = 0.0, y = 0.0, z = 0.0;
    V3 operator+(const V3& o) const noexcept { return {x + o.x, y + o.y, z + o.z}; }
    V3 operator-(const V3& o) const noexcept { return {x - o.x, y - o.y, z - o.z}; }
    V3 operator*(double k) const noexcept { return {x * k, y * k, z * k}; }
    V3& operator+=(const V3& o) noexcept { x += o.x, y += o.y, z += o.z; return *this; }
};
inline double dot(const V3& a, const V3& b) noexcept { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(const V3& a, const V3& b) noexcept { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double length(const V3& a) noexcept { return std::sqrt(dot(a, a)); }
inline V3 normalized(const V3& a) noexcept { const double l = length(a); return l > 0.0 ? a * (1.0 / l) : V3{}; }

// -- the warhead ------------------------------------------------------------------------------------------------

/// A share of the fragments thrown between two angles from the missile's nose (degrees).
struct Zone {
    double fromDeg = 0.0, toDeg = 0.0, share = 0.0;
};

/// A warhead as its card and one rule make it (every missile's the same way; estimates said so in warheadOf):
/// its charge drives its case - preformed cubes, or rods for a rod warhead - out at Gurney's cylinder velocity.
struct Warhead {
    enum class Kind { Fragmentation, Rod } kind = Kind::Fragmentation;
    double totalKg = 0.0, explosiveKg = 0.0;
    double gurneyMs = 2600.0;      ///< the explosive's sqrt(2E)
    double tntKg = 0.0;            ///< the charge as TNT for its blast (its Gurney energy against TNT's)
    double caseKg = 0.0;           ///< what it throws: its fragments' (or rods') metal
    double diameterM = 0.0, lengthM = 0.0, caseThicknessM = 0.0;
    double fragmentKg = 0.0;       ///< each fragment's (a cube as thick as its case) or rod's (that thick, as long as it)
    double fragmentAreaM2 = 0.0;   ///< each one's mean presented area, tumbling (a convex body's: a quarter of its surface)
    int fragments = 0;
    double speedMs = 0.0;          ///< thrown at, relative to the missile
    std::vector<Zone> zones;       ///< where (from the nose), each zone's share
};

/// Gurney's cylinder: a case of mass M round a charge C thrown at sqrt(2E) (M/C + 1/2)^-1/2.
double gurneyCylinder(double gurneyMs, double caseKg, double chargeKg) noexcept;
/// The warhead a store card describes (nullopt: the card has none).
std::optional<Warhead> warheadOf(const StoreCard& card);

// -- the blast --------------------------------------------------------------------------------------------------

/// A free-air burst's side-on peak overpressure (Pa) and positive impulse (Pa s) at a range from W kg of TNT, in air
/// at this pressure and temperature: Kingery-Bulmash's fits (Swisdak 1994, a hemispherical surface burst, taken at W/2
/// for one in free air) scaled for altitude by Sachs.
struct Blast {
    double overpressurePa = 0.0, impulsePas = 0.0;
};
Blast blastAt(double tntKg, double rangeM, double ambientPa, double ambientK) noexcept;
/// A face's peak pressure where the shock meets it head on: the side-on's reflected (ideal gas, Rankine-Hugoniot).
double reflected(double overpressurePa, double ambientPa) noexcept;

// -- penetration ------------------------------------------------------------------------------------------------

/// A steel fragment through an aluminium (2024-T3) plate: its speed after (0: stopped), by Project THOR's residual
/// velocity equation (BRL, TR-47, 1961) - the plate's thickness (m), its presented area (m^2), mass (kg), speed (m/s)
/// and the angle off the plate's normal (rad).
double thorResidual(double plateM, double areaM2, double massKg, double speedMs, double obliquityRad) noexcept;

// -- the airframe -----------------------------------------------------------------------------------------------

/// An aircraft's damage data (hangar's <type>.damage.json).
struct Airframe {
    struct Section {
        std::string name;
        double massKg = 0.0;
        V3 cg;
        std::array<double, 6> inertia{}; ///< ixx, iyy, izz, ixy, ixz, iyz about its cg
        struct Tank { int index = 0; std::string name; double capacityKg = 0.0; V3 centre; };
        std::vector<Tank> tanks;
        std::vector<int> stations;        ///< the store stations on it (their stores part with it)
        double normalShare = 0.0, sideShare = 0.0;
        std::optional<V3> normalCp, sideCp;
        bool hasRoot = false;
        V3 root;
        std::vector<V3> axes;             ///< what its root bends about
        std::vector<double> strengthNm;   ///< about each, built to carry
        double skinAreaM2 = 0.0, skinKg = 0.0;
    };
    struct Panel {
        int section = -1;
        bool surface = true;
        std::array<V3, 4> corners{};      ///< a surface panel's
        double thicknessM = 0.0;          ///< its section's depth there (a surface's)
        V3 centre, half;                  ///< a body box's
        double areaM2 = 0.0;
        V3 middle() const noexcept;
    };
    struct Component {
        std::string kind, shape, name;
        int section = -1;
        V3 centre, half;
        int engine = -1, tank = -1;
        std::string channel;
        double fuelKg = 0.0;
    };
    std::string aircraft;
    V3 originDesign;               ///< the origin in its design's (and JSBSim's structural) frame: x aft, y right, z up
    double designMassKg = 0.0;
    double nLimit[2] = {0.0, 0.0}, nUltimate[2] = {0.0, 0.0};
    std::vector<Section> sections;
    std::vector<Panel> panels;
    std::vector<Component> components;
    int section(const std::string& name) const noexcept;
    /// A section's skin's thickness (m): its skin's mass over its area, in aluminium.
    double skinThicknessM(int section) const noexcept;
};
/// Read an aircraft's damage data (nullopt and `error` said: none, or it cannot be read).
std::optional<Airframe> readAirframe(const std::filesystem::path& file, std::string& error);

// -- the impact -------------------------------------------------------------------------------------------------

/// The detonation, and the target as the fragments find it (ECEF; the target's attitude body to ECEF, rows).
struct Burst {
    V3 at, velocity;  ///< the warhead's centre and the missile's velocity (m/s)
    V3 axis;          ///< the missile's nose
};
struct Target {
    V3 at, velocity;                ///< its model's origin and velocity
    std::array<V3, 3> bodyToEcef{}; ///< the body axes (x, y, z) in ECEF
    double airPressurePa = 101325.0, airTemperatureK = 288.15, airDensity = 1.225;
};

/// What the burst does to the target.
struct Impact {
    struct ComponentHits {
        int component = -1;
        int hits = 0;
        double energyJ = 0.0;  ///< the hits' kinetic energy on striking it (relative to it)
        double fastestMs = 0.0;
        double areaM2 = 0.0;   ///< the hits' presented areas (what they hole it by)
    };
    struct PanelHits {
        int panel = -1;
        int hits = 0, perforations = 0;
        double holeAreaM2 = 0.0;  ///< the perforations' (each its fragment's presented area)
        double overpressurePa = 0.0, impulsePas = 0.0; ///< the blast's on its face (reflected as far as it faces it)
        V3 momentumNs;            ///< what the fragments striking it gave it (body axes)
    };
    double missM = 0.0;              ///< the burst from the target's origin
    V3 fromBody;                     ///< the burst, seen from the origin (body axes, unit)
    int fragmentsThrown = 0, fragmentsStriking = 0;
    V3 impulseBody;                  ///< what the fragments striking and the blast push it by (N s, body axes)
    V3 angularImpulseBody;           ///< ... and turn it by, about its origin (N m s, body axes)
    std::vector<ComponentHits> components; ///< those struck
    std::vector<PanelHits> panels;         ///< those struck, or loaded by the blast past kBlastSeenPa
};
constexpr double kBlastSeenPa = 1000.0; ///< a panel's blast kept from this (1 kPa: a hundredth of an atmosphere)
/// The fragments' flight (each drawn from `rng`: its zone, its direction in it) and the blast, across the target.
Impact assess(const Warhead& warhead, const Burst& burst, const Airframe& airframe, const Target& target, Rng& rng);

} // namespace fsim::ms::damage
