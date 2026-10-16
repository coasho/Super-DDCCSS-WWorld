#pragma once

// Scenario files on the session world (design 9.13). The parser, dumper and
// this apply are shared by the C++ SDK (fsim::applyScenario) and the C ABI.
// The runtime applies the native recipe (session/ScenarioRecipe.h); the public
// document maps onto it, its world and batch-environment sections aside.

#include "fsim/ScenarioDocument.h"
#include "session/ScenarioRecipe.h"

#include <cstdint>
#include <vector>

namespace fsim::session {

class World;

/// The document's native recipe: its environment, world-wide effects and vehicles, field for field.
ScenarioRecipe scenarioRecipe(const Scenario& scenario);

/// See fsim::applyScenario; returns the created vehicle ids.
std::vector<std::uint32_t> applyScenario(World& world, const ScenarioRecipe& recipe);
std::vector<std::uint32_t> applyScenario(World& world, const Scenario& scenario);

/// Only the environment and the world-wide effects (what a VecEnv takes from a scenario).
void applyScenarioWorld(World& world, const ScenarioRecipe& recipe);
void applyScenarioWorld(World& world, const Scenario& scenario);

} // namespace fsim::session
