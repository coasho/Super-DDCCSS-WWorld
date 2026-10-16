#pragma once

// What a hit leaves an aircraft with, and what that does as it flies on (MS-7g's damage part, G4 - G6): each holed
// tank's leak and fire, an engine out, a control channel lost, the crew; each section's skin torn or holed, so its
// share of the lift gone and its root weaker; and, each step, each section's root bending from its share of the
// aerodynamic force and its own inertia against what is left of its strength - a section that cannot carry it
// separates. Pure: the world flies the consequences (session/World) and draws the chances from the vehicle's own
// damage stream. Every number not from a source is an estimate, said so.

#include "simulation/mission_systems/damage/Damage.h"

#include <map>
#include <memory>

namespace fsim::ms::damage {

// The estimates (no public source gives them for a modern airframe):
constexpr double kPanelYieldPa = 440e6;    ///< 2024-T3's ultimate strength (MMPDS)
constexpr double kPanelElongation = 0.18;  ///< and its elongation (MMPDS)
constexpr double kPanelStraining = 0.05;   ///< the share of a panel that stretches before it tears (an estimate)
constexpr double kIgnitionJ = 16e3;        ///< P(fire | a fragment into a tank) = 1 - exp(-E / kIgnitionJ) at sea level (an estimate)
constexpr double kRamFactor = 5.0;         ///< a full tank's hole: the fragment's area times this, petalled by hydrodynamic ram (an estimate)
constexpr double kRamSpeedMs = 1000.0;     ///< ... above this speed
constexpr double kSpillShare = 0.05, kSpillMostKg = 50.0; ///< what the ram throws out at once: a share of the tank, at most (estimates)
constexpr double kEngineKill = 0.4;        ///< P(engine out | a hit through its casing): the middle of 0.3 - 0.6 (an estimate)
constexpr double kEngineFire = 0.25;       ///< P(its fire | it is out) (an estimate)
constexpr double kControlKill = 0.5;       ///< P(a control channel lost | its actuator hit) (an estimate)
constexpr double kCrewKill = 0.6;          ///< P(incapacitated | a hit faster than kCrewSpeedMs) (an estimate)
constexpr double kCrewSpeedMs = 300.0;
constexpr double kHoleCd = 0.7;            ///< a petalled hole's discharge coefficient (0.6 - 0.8)
constexpr double kHeadM = 0.3;             ///< the fuel's head over a hole (an estimate)
constexpr double kSuctionCp = 0.5;         ///< the flow's suction over a hole, of the dynamic pressure (an estimate)
constexpr double kFuelDensity = 800.0;     ///< kerosene
/// A fire held in the airstream behind torn structure burns what mixes with the air in its wake, at most this much
/// (kg/s); the rest of a leak leaves as mist (an estimate: no published blow-off or burning-rate correlation)
constexpr double kBurnMostKgS = 5.0;
constexpr double kDamagedCd = 0.3;         ///< a torn panel's or hole's added drag on its area: the structure under it
                                           ///< exposed, not a plate across the flow (an estimate)
constexpr double kCompressionExponent = 1.5; ///< a root's strength left: (1 - f)^1.5 - its compression side (an estimate: 1.5 - 2)
/// A blast's impulse on a section, far shorter than its bending mode's period, bends its root by the impulse's moment
/// times the mode's angular frequency (a single degree of freedom's response to an impulse); the mode's frequency from
/// its tip bending kTipDeflection of its reach at its ultimate load: omega = sqrt(n_ult g / delta) (an estimate - a
/// fighter's wing some 4 Hz)
constexpr double kTipDeflection = 0.05;
/// Fragment holes closer than a few diameters link by cracks: a panel's share lost to n perforations
/// 1 - exp(-n kCrackLink a / A) (a each one's area, A the panel's) - the holes' own area times kCrackLink where they are
/// sparse, the whole panel where they are dense (an estimate: Ball's fragment-induced cracking, its linkage some 4 - 5
/// hole diameters)
constexpr double kCrackLink = 20.0;

/// A torn panel's impulse (Pa s): as much as its plastic work takes, t sqrt(2 rho sigma eps k).
double tearImpulse(double thicknessM) noexcept;

/// What changed, for the world to fly and to record.
struct Change {
    enum class Kind { Fire, FireOut, EngineOut, ControlLost, CrewLost, PanelTorn, Separated } kind = Kind::Fire;
    int component = -1;   ///< the component (Fire, EngineOut, ControlLost, CrewLost)
    int tank = -1, engine = -1;
    std::string channel;  ///< ControlLost's
    int panel = -1;       ///< PanelTorn's
    int section = -1;     ///< PanelTorn's, Separated's
    double spillKg = 0.0; ///< Fire's fuel thrown out at once
};

/// The flight state the consequences read (body axes; SI).
struct Flight {
    V3 aeroForceN;              ///< the aerodynamic force on the whole aircraft
    V3 specificForceMs2;        ///< every force but gravity over the mass: what an accelerometer at the CG reads
    V3 rateRadS, rateDotRadS2;  ///< body rates and their rates
    double dynamicPressurePa = 0.0, airPressurePa = 101325.0;
    std::vector<double> tankKg; ///< each JSBSim tank's fuel now
};

class VehicleDamage {
public:
    explicit VehicleDamage(std::shared_ptr<const Airframe> airframe);
    const Airframe& airframe() const noexcept { return *airframe_; }

    struct Tank { double holeM2 = 0.0; bool burning = false; int component = -1; };
    struct Section {
        double strength = 1.0, lift = 1.0;
        bool separated = false;
        double tornM2 = 0.0, holeM2 = 0.0, removed = 0.0;
        double kick = 0.0; ///< the last hit's peak root bending against what strength it had left (1: it fails)
    };
    const std::map<int, Tank>& tanks() const noexcept { return tanks_; }
    const std::vector<Section>& sections() const noexcept { return sections_; }
    bool engineOut(int engine) const noexcept;
    bool crewLost() const noexcept { return crewLost_; }
    const std::vector<std::string>& lostChannels() const noexcept { return lostChannels_; }
    bool panelTorn(int panel) const noexcept { return panel >= 0 && panel < static_cast<int>(torn_.size()) && torn_[static_cast<std::size_t>(panel)]; }

    /// A hit: what its fragments and blast break, its chances drawn from `rng`; what changed.
    std::vector<Change> apply(const Impact& impact, double airPressurePa, Rng& rng);

    /// Each holed tank's leak now (kg/s): its holes' orifice flow, the fuel's head under the load factor and the flow's
    /// suction over it.
    double leakKgS(int tank, double contentsKg, double loadFactor, double dynamicPressurePa) const;

    /// The force and moment (body axes, about the origin) the damage takes off the aerodynamic one: each section's
    /// lost share of it at its centre of pressure, and the drag of what is torn and holed.
    void aeroLoss(const Flight& flight, V3& forceN, V3& momentNm) const;

    /// Each section's root bending now against its strength left: those that fail (and are marked separated).
    std::vector<Change> loads(const Flight& flight);
    /// A section marked separated outright (false: it has no root, or is gone already).
    bool separate(int section);
    /// A section's bending about each of its root's axes now (N m).
    std::vector<double> bending(int section, const Flight& flight) const;

private:
    std::shared_ptr<const Airframe> airframe_;
    std::map<int, Tank> tanks_;
    std::vector<Section> sections_;
    std::vector<bool> torn_;
    std::vector<int> engines_out_;
    std::vector<std::string> lostChannels_;
    bool crewLost_ = false;
    std::vector<double> span_; ///< each section's reach from its root (for where its damage is)
    void weaken(int section, const V3& where, double removedM2);
};

} // namespace fsim::ms::damage
