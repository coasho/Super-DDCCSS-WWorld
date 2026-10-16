#pragma once

// fsim SDK: scenario files (design 9.13). A JSON document describes a world,
// its environment, and the vehicles to create with their initial commands
// and effects, so an experiment is data rather than code:
//
//   auto scenario = fsim::loadScenario("dogfight.json");
//   fsim::World world(scenario.world);
//   auto vehicles = fsim::applyScenario(world, scenario);
//   for (;;) world.step();
//
// The document's format, values and reading/writing: fsim/ScenarioDocument.h (included here).

#include "fsim/Export.h"
#include "fsim/ScenarioDocument.h"
#include "fsim/VecEnv.h"
#include "fsim/World.h"

#include <vector>

namespace fsim {

/// Apply the environment, the world-wide effects and create every vehicle
/// (with its effects and initial command). Returns the vehicles in file
/// order; throws fsim::Error when a vehicle cannot be created or a behaviour
/// target is unknown (vehicles created so far remain).
FSIM_API std::vector<Vehicle> applyScenario(World& world, const Scenario& scenario);

/// VecEnv options from a scenario: the "world" section, the "vecenv" section
/// and the first vehicle's type and initial state as the episode template.
/// `scenarioPath` is set when the scenario came from a file, so the VecEnv
/// also applies its environment and world-wide effects.
FSIM_API VecEnvOptions vecEnvOptions(const Scenario& scenario);

} // namespace fsim
