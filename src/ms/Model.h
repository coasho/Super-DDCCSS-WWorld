#pragma once

// What the payloads beyond the radar and the store stations share (ADR-30, MS-9 to MS-11): the world as a sensor
// meets it - the other entities, the transmitters among them, the decoys in the air - and one interface for a
// payload's model: the task it is given, its step with the world, the contacts it holds, its report, what it
// radiates. The host (ms/Host) owns the models, their activities and their grants; the world (session/World) builds
// the scene and flies what the models release into it.

#include "fsim/EnvironmentState.h"
#include "fsim/MissionSystems.h"
#include "fsim/Rng.h"
#include "fsim/VehicleState.h"
#include "ms/Cards.h"
#include "ms/radar/Radar.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace fsim::ms {

/// A transmitter in the world as a receiver, or a victim, meets it: a radar's scan, a seeker, a jammer, a terminal.
struct Emission {
    std::uint32_t entity = 0;  ///< its carrier (a missile, for its seeker)
    std::uint16_t payload = 0;
    EmitterMode mode = EmitterMode::Search;
    double frequencyHz = 10e9;
    double bandwidthHz = 1e6;  ///< a jammer's: what its power is spread over
    double powerW = 0.0;       ///< average, into its antenna
    double peakPowerW = 0.0;   ///< a pulse's (0: it does not pulse - its average is all there is)
    double peakGainDbi = 0.0;  ///< towards where its main beam goes
    double sidelobeDbi = -5.0; ///< everywhere else
    /// Where its main beam goes, in its carrier's body axes: the volume it scans (a beam that stays: its beam).
    double azimuthCentreRad = 0.0, azimuthHalfWidthRad = 0.0, elevationCentreRad = 0.0, elevationHalfWidthRad = 0.0;
    bool allRound = false;     ///< every azimuth (a rotodome, an omnidirectional antenna)
    std::uint32_t target = 0;  ///< the entity it is locked on (0: none) - the simulation's knowledge
    const sim::VehicleState* state = nullptr; ///< its carrier's
    /// The gain (dBi) it radiates with towards a place (ECEF).
    double gainTowardsDbi(const double placeEcef[3]) const noexcept;
};

/// A decoy in the air (MS-9a): a chaff cloud or a flare, flown by the world.
struct Decoy {
    enum class Kind : std::uint8_t { Chaff, Flare };
    std::uint32_t id = 0;       ///< kDecoyIds + its count
    Kind kind = Kind::Chaff;
    std::uint32_t owner = 0;    ///< who dispensed it
    double bornS = 0.0;
    sim::VehicleState state;    ///< where it is and how it moves (the fields a sensor reads)
    double sigmaM2 = 0.0;       ///< a chaff cloud's radar cross section now (its bloom and its fall-off)
    double intensityWSr = 0.0;  ///< a flare's radiant intensity now, in the 3-5 um band
};

/// What a payload's model sees of the world: a radar's scene, and what the other sensors need besides.
class Scene : public radar::Scene {
public:
    virtual const std::vector<Emission>& emissions() const = 0;
    /// The decoy behind an id a `targets` call gave (null: an entity).
    virtual const Decoy* decoy(std::uint32_t id) const { (void)id; return nullptr; }
    virtual Role role(std::uint32_t id) const { (void)id; return Role::Aircraft; }
    /// Who released a weapon (0: not one).
    virtual std::uint32_t launcher(std::uint32_t id) const { (void)id; return 0; }
    virtual const sim::EnvironmentState& environment() const = 0;
    /// The terrain, or the earth's curve, stands between the carrier and another entity: the radio path's loss at
    /// 10 GHz past free space by more than a ridge's edge gives (20 dB).
    virtual bool masked(std::uint32_t other, double rangeM) const {
        const double freeSpace = 20.0 * std::log10(4.0 * 3.14159265358979323846 * std::max(rangeM, 1.0) * 10e9 / 299792458.0);
        return pathLossDb(other, 10e9) - freeSpace > 20.0;
    }
};

/// A burst leaving a gun this step (MS-9c): the world flies its rounds.
struct GunBurst {
    std::uint16_t payload = 0;
    int rounds = 0;
    double muzzleMs = 1000.0;
    double projectileKg = 0.1;
    double calibreM = 0.02;
    double dispersionRad = 0.002; ///< one standard deviation an axis
    double explosiveKg = 0.0;     ///< a shell's filling (0: a solid shot)
    double dragK = 0.0;           ///< its retardation at sea level: dv/dt = -dragK (rho / rho0) v^2 (1/m)
    double elevationRad = 0.0;    ///< the barrel's line above the body's x axis (a fixed gun's harmonisation)
};

/// Decoys leaving a dispenser this step (MS-9a).
struct Dispensed {
    std::uint16_t payload = 0;
    Decoy::Kind kind = Decoy::Kind::Chaff;
    int count = 0;
    double peak = 0.0;      ///< each one's: a chaff bundle's bloomed cross section (m^2), a flare's peak intensity (W/sr)
    double burnS = 0.0;     ///< a flare's burn time; a chaff cloud's bloom time
};

/// One payload's model beyond its state machine. Stepped with its vehicle's sensors, on the workers: it reads the
/// scene, and writes nothing but itself.
class Model {
public:
    virtual ~Model() = default;
    /// A task for it, the payload operating: None and it has begun; else why not.
    virtual Reason begin(const PayloadTask& task, double now) { (void)task, (void)now; return Reason::InvalidParameter; }
    /// Its task ends (canceled, superseded, the payload no longer operating).
    virtual void stop(double now) { (void)now; }
    /// Its task is over by itself: how (GoalReached, Expended, ContactLost ...); None while it runs.
    virtual Reason finished() const { return Reason::None; }
    /// Its task's progress, 0..100 (NaN: none to measure).
    virtual double percent() const { return std::numeric_limits<double>::quiet_NaN(); }
    /// The contact of another of the vehicle's sensors its task follows (0: none); `held` is that contact now, null
    /// once its sensor has let it go.
    virtual std::uint64_t follows() const { return 0; }
    virtual void followed(const Contact* held) { (void)held; }
    /// The payload operating: a step of its model, to `now`.
    virtual void step(double now, const sim::VehicleState& own, const Scene& scene, Rng& rng) { (void)now, (void)own, (void)scene, (void)rng; }
    /// The payload stopped operating (off, standby, failed): what it held is gone.
    virtual void idle(double now) { (void)now; }
    virtual void contacts(std::vector<Contact>& out) const { (void)out; }
    virtual void report(std::vector<PayloadValue>& out) const { (void)out; }
    /// What it radiates now (false: nothing).
    virtual bool emission(Emission& out) const { (void)out; return false; }
    /// What left it since last asked (the world flies them).
    virtual bool takeBurst(GunBurst& out) { (void)out; return false; }
    virtual bool takeDispensed(std::vector<Dispensed>& out) { (void)out; return false; }
    /// The mass it has shed since last asked (kg): rounds fired, cartridges gone.
    virtual double takeShedKg() { return 0.0; }
};

/// The model a card's payload has (null: its family has none beyond its state - a store station, a radar).
/// `count`: the fit's (how many of it the aircraft carries: a dispenser's sets, a gun's barrels are its card's).
std::unique_ptr<Model> makeModel(const Card& card, std::uint16_t count);

// --- shared geometry (ECEF, body axes) -----------------------------------------------------------------------------

namespace geom {
inline constexpr double kPi = 3.14159265358979323846;
inline double dot3(const double a[3], const double b[3]) noexcept { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline double norm3(const double a[3]) noexcept { return std::sqrt(dot3(a, a)); }
/// An ECEF vector in a vehicle's body axes (its rotation's columns are its body axes in ECEF).
inline void toBody(const sim::VehicleState& s, const double ecef[3], double body[3]) noexcept {
    const double* R = s.rotationBodyToEcef;
    for (int c = 0; c < 3; ++c) body[c] = R[c] * ecef[0] + R[3 + c] * ecef[1] + R[6 + c] * ecef[2];
}
inline void toEcef(const sim::VehicleState& s, const double body[3], double ecef[3]) noexcept {
    const double* R = s.rotationBodyToEcef;
    for (int r = 0; r < 3; ++r) ecef[r] = R[3 * r] * body[0] + R[3 * r + 1] * body[1] + R[3 * r + 2] * body[2];
}
/// A vehicle's velocity in ECEF (m/s).
inline void velocityEcef(const sim::VehicleState& s, double out[3]) noexcept { toEcef(s, s.velocityBodyMs, out); }
/// From one vehicle to a place: the unit line of sight (ECEF) into `los`; the range (m).
inline double lineOfSight(const sim::VehicleState& from, const double toEcefM[3], double los[3]) noexcept {
    for (int k = 0; k < 3; ++k) los[k] = toEcefM[k] - from.positionEcef[k];
    const double range = norm3(los);
    if (range > 0.0)
        for (int k = 0; k < 3; ++k) los[k] /= range;
    return range;
}
/// A body-axes direction's azimuth (from the nose, right positive) and elevation (above the x-y plane).
inline void angles(const double body[3], double& azimuth, double& elevation) noexcept {
    azimuth = std::atan2(body[1], body[0]);
    elevation = std::atan2(-body[2], std::hypot(body[0], body[1]));
}
/// Whether a body-axes direction lies in a volume about a centre.
inline bool within(double azimuth, double elevation, double azCentre, double azHalf, double elCentre, double elHalf) noexcept {
    return std::fabs(std::remainder(azimuth - azCentre, 2.0 * kPi)) <= azHalf && std::fabs(elevation - elCentre) <= elHalf;
}
/// The standard atmosphere's density over the sea level's, at a height (m): the troposphere's, then the isothermal layer's.
inline double densityRatio(double heightM) noexcept {
    const double h = std::max(heightM, 0.0);
    return h < 11000.0 ? std::pow(1.0 - 2.25577e-5 * h, 4.2559) : 0.2971 * std::exp(-(h - 11000.0) / 6341.6);
}
} // namespace geom

} // namespace fsim::ms
