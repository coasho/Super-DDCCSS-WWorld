# session: the simulation runtime (ARCH-04; docs/architecture/modules/simulation/plan.md) - the authoritative World
# and its native services, split by the state each file owns:
#   world/     the World: vehicles, stepping, environment, frames and geometry, engagements and damage, the native
#              scenario recipe and its application
#   services/  native Agent/C2/MA services: MaCore's tasks and consent, service bindings, reports, Agent rules
#   stores/    store-to-flight binding state
# Modeled DMS is its own target (simulation/communications/dms) and the public facade is simulation_api
# (simulation/api); target name and alias are unchanged.
add_library(fsim_session STATIC
    world/World.h
    world/World.cpp
    world/WorldDamage.cpp
    world/WorldEngagements.cpp
    world/WorldWeaponControls.cpp
    world/Frames.cpp
    world/Geometry.cpp
    world/Marshalls.cpp
    world/Intercepts.cpp
    world/Navigation.cpp
    world/PerformanceProfile.cpp
    world/StateData.cpp
    world/Scenario.h
    world/ScenarioRecipe.h
    world/Scenario.cpp
    services/AgentRules.h
    services/AgentRules.cpp
    services/MaCore.h
    services/MaCore.cpp
    services/MaConsentLedger.h
    services/MaServiceBindings.h
    services/MaServiceBindings.cpp
    services/MaReports.h
    services/MaReports.cpp
    stores/StoreFlightBinding.h
    stores/StoreFlightBinding.cpp)
add_library(fsim::session ALIAS fsim_session)
fsim_include_view(fsim_session)
target_link_libraries(fsim_session PUBLIC fsim::contracts fsim::modeled_dms fsim::ms fsim::control fsim::effects
    fsim::comm fsim::ipc fsim::sim fsim::io fsim::core)
fsim_set_warnings(fsim_session)
