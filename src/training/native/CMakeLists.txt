# Training's native implementation (ARCH-05; docs/architecture/modules/training/plan.md), over the simulation runtime
# and its lower facade (simulation_api); it knows nothing about rendering and nothing of the final SDK assembly, which
# links both targets whole.

# env: the legacy flight VecEnv (vecenv/: environments, tasks, observations, the plugin registry, scenario options)
# and, separately, the independent mission episode's native owner (episodes/runtime/: one World per episode, its
# generations, holds, reset and seed streams). Target name and alias are unchanged.
add_library(fsim_env STATIC
    episodes/runtime/MissionEpisode.h
    episodes/runtime/MissionEpisode.cpp
    vecenv/Observation.h
    vecenv/Observation.cpp
    vecenv/Registry.h
    vecenv/Registry.cpp
    vecenv/Scenario.h
    vecenv/ScenarioVecEnv.cpp
    vecenv/Task.h
    vecenv/Task.cpp
    vecenv/VecEnv.h
    vecenv/VecEnv.cpp)
add_library(fsim::env ALIAS fsim_env)
fsim_include_view(fsim_env DIRECTORIES vecenv episodes/runtime)
# Mission episodes map public construction options through the lower simulation facade (simulation_api).
target_link_libraries(fsim_env PUBLIC fsim::simulation_api fsim::session fsim::control fsim::sim fsim::io fsim::core)
fsim_set_warnings(fsim_env)

# mission: the mission training implementation - configuration and its strict JSON (mission/), the gateway with its
# snapshots, requests, receipts and costs (mission/), mission batches and their final documents (batch/), and the public
# mission episode facade (episodes/api/), distinct from the native episode owner above. It consumes the World and Agent
# through simulation_api. The SDK links it whole and exports its public classes, defined here once; it is compiled as
# the SDK compiles its sources (FSIM_BUILDING, hidden visibility).
add_library(fsim_training_mission STATIC
    mission/MissionConfig.h
    mission/MissionConfig.cpp
    mission/MissionJson.h
    mission/MissionJson.cpp
    mission/MissionGateway.h
    mission/MissionGateway.cpp
    batch/MissionBatch.cpp
    episodes/api/MissionEpisodeImpl.h
    episodes/api/MissionEpisode.cpp)
add_library(fsim::mission ALIAS fsim_training_mission)
fsim_include_view(fsim_training_mission DIRECTORIES mission episodes/api)
target_link_libraries(fsim_training_mission PUBLIC fsim::env fsim::simulation_api fsim::session fsim::core)
target_compile_definitions(fsim_training_mission PRIVATE FSIM_BUILDING)
set_target_properties(fsim_training_mission PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON)
fsim_set_warnings(fsim_training_mission)
