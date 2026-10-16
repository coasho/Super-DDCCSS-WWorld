#pragma once

// The native scenario recipe: what the runtime applies to a session::World - its environment, its world-wide effects
// and its vehicles with their effects and initial commands (design 9.13). It is the runtime's own input (ARCH-02): the
// public scenario document (fsim/ScenarioDocument.h) maps onto it field for field (session::scenarioRecipe), and its
// "world" and "vecenv" sections - construction options and the batch environment's recipe - are not the runtime's.

#include "fsim/Control.h"
#include "fsim/EnvironmentState.h"
#include "fsim/InitialConditions.h"

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace fsim::session {

/// A built-in effect by id with named parameters (effects::createBuiltinEffect).
struct ScenarioEffect {
    std::string id;
    std::vector<std::pair<std::string, double>> params;
};

/// A vehicle as the runtime creates it: its spec's name, type, model, control divider and initial state; `count`
/// instances (> 1 names them "<name>-1".."<name>-N", `spacingM` abreast, to the right of the heading); its initial
/// command (a behaviour's target by vehicle name, resolved when applied) and its effects.
struct ScenarioVehicleRecipe {
    std::string name;
    std::string type;
    std::string model;
    unsigned controlDivider = 1;
    sim::InitialConditions initial;
    unsigned count = 1;
    double spacingM = 200.0;
    std::optional<control::Command> command;
    std::string commandTarget;
    std::vector<ScenarioEffect> effects;
};

struct ScenarioRecipe {
    std::string source;                  ///< the document's name, for messages
    bool hasEnvironment = false;
    sim::EnvironmentState environment;   ///< applied when `hasEnvironment` (epochUtcSeconds 0 = leave the clock alone)
    std::vector<ScenarioEffect> effects; ///< for every vehicle, present and future
    std::vector<ScenarioVehicleRecipe> vehicles;
};

} // namespace fsim::session
