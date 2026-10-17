# simulation_api: the private lower simulation facade (ARCH-04) - the existing implementations of the public World,
# Agent and recording classes (include/fsim), and the public-to-native construction mapping. (fsim::applyScenario stays
# in the SDK assembly: its public header, fsim/Scenario.h, also declares training's vecEnvOptions.) Mission and
# training code consume it below the final SDK assembly, which links it whole and exports its classes; each exported
# method has this one definition. Compiled as the SDK compiles its sources (FSIM_BUILDING, hidden visibility).
add_library(fsim_simulation_api STATIC
    WorldOptions.h
    WorldOptions.cpp
    World.cpp
    Agent.cpp
    Recording.cpp)
add_library(fsim::simulation_api ALIAS fsim_simulation_api)
fsim_include_view(fsim_simulation_api)
target_link_libraries(fsim_simulation_api PUBLIC fsim::session fsim::ipc fsim::core)
target_compile_definitions(fsim_simulation_api PRIVATE FSIM_BUILDING)
set_target_properties(fsim_simulation_api PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON)
fsim_set_warnings(fsim_simulation_api)
