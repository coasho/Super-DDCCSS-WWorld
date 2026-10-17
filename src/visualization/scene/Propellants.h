#pragma once

#include <algorithm>

namespace fsim::world {

/// A solid motor's propellant class as its exhaust is drawn (world::Exhaust's plume, world::Airflow's smoke and ice
/// contrail): its nozzle's exit and its exhaust's composition through NASA's chemical equilibrium code (CEA, McBride
/// and Gordon, NASA RP-1311: tools/models/propellants/cea_classes.py, its numbers in tools/models/propellants/classes.json, which
/// test_vehicle_visuals holds these to). One composition a class, the same for every nation's missiles - aluminised
/// AP 68 % / Al 18 % / HTPB 14 %, reduced smoke AP 86 % / HTPB 14 % - at a 10 MPa chamber (the hangar designs' own,
/// their thrust over their throats: 9 - 10 MPa) through their nozzles' area ratio (4.94), shifting equilibrium. The
/// reduced class's AP share is the assumption that matters: 84 - 88 % moves its exit 1,477 - 1,847 K (its chamber's
/// pressure, 7 - 14 MPa, a few kelvin).
struct Propellant {
    double exitK;            ///< the gas at the nozzle's exit (an aluminised motor's held at alumina's freezing point)
    double gamma;            ///< the expansion's effective ratio of specific heats (an aluminised motor's two phases)
    double exitMach;
    double molarMass;        ///< the exit gas's, g/mol
    double exitPressure;     ///< the exit's static pressure over sea level's at the full thrust (10 MPa)
    double water;            ///< kg of water per kg of propellant once its hydrogen has burnt in the air
    double afterburning;     ///< J per kg of propellant its H2 and CO release burning in the air (lower heating values)
    double hcl;              ///< mass fraction of HCl in its exhaust
    double alumina;          ///< mass fraction of alumina (liquid and solid)
    double ispS;             ///< delivered specific impulse, s: 95 % of the ideal at the exit's pressure (an assumption:
                             ///< a tactical motor's combustion and nozzle efficiencies)
    bool computed;           ///< from CEA; false: an estimate
};

/// By class: 0 minimum smoke, 1 reduced smoke, 2 aluminised (the manifest's order). The minimum-smoke class is an
/// estimate - no design uses one, and a nitramine double base's ingredients (nitrocellulose, nitroglycerine, HMX) are
/// not in CEA's library: a cooler, water-rich exhaust with no HCl or metal, otherwise as the reduced class's.
inline constexpr Propellant kPropellants[3] = {
    {1400.0, 1.22, 2.79, 25.0, 3.093, 0.30, 3.8e6, 0.0, 0.0, 0.95 * 231.6, false},
    {1648.3, 1.2037, 2.79, 25.422, 3.093, 0.3319, 2.273e6, 0.2669, 0.0, 0.95 * 231.6, true},
    {2327.0, 1.1354, 2.877, 26.209, 3.578, 0.2898, 5.807e6, 0.2091, 0.3396, 0.95 * 240.7, true},
};

inline const Propellant& propellant(int smokeClass) { return kPropellants[std::clamp(smokeClass, 0, 2)]; }

} // namespace fsim::world
