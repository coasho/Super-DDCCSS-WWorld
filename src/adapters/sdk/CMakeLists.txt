# fsim.dll: the public SDK (C++ pimpl class + versioned C ABI). Internal
# static libraries are linked in; only fsim/*.h symbols are exported.
# The World, Agent and recording class implementations are simulation_api's (src/simulation/api) and the mission
# training implementation is training's (src/training/native, fsim_training_mission); both are linked whole below and
# exported from here as before.
# ARCH-07: the final assembly is thin - cpp/ holds the C++ compatibility facade (the VecEnv pimpl, applyScenario's
# delegation), c/ the sized C ABI adapters with their handles, error and principal guards. The assembly compiles with
# its own private headers (c/) and the include views of the modules it links, never the src root.
add_library(fsim SHARED cpp/VecEnv.cpp cpp/Scenario.cpp
    c/fsim_c.cpp c/fsim_vecenv_c.cpp c/fsim_recording_c.cpp c/fsim_world_c.cpp c/fsim_scenario_c.cpp
    c/fsim_environment_c.cpp c/fsim_control_c.cpp c/fsim_mission_c.cpp c/fsim_links_c.cpp c/fsim_mission_env_c.cpp
    c/VecEnvHandles.h c/LastError.h c/CAbiSupport.h)
add_library(fsim::sdk ALIAS fsim)

target_include_directories(fsim
    PUBLIC  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/c)
# Whole-archive: built-in controllers/effects are reached only through the
# registry or by trainer code, so nothing in the DLL would otherwise pull
# their objects in and export them.
target_link_libraries(fsim PRIVATE
    "$<LINK_LIBRARY:WHOLE_ARCHIVE,fsim::mission,fsim::env,fsim::simulation_api,fsim::session,fsim::modeled_dms,fsim::ms,fsim::control,fsim::effects,fsim::comm,fsim::ipc>"
    fsim::sim fsim::io fsim::core fsim::platform)
target_compile_definitions(fsim PRIVATE FSIM_BUILDING FSIM_VERSION_STRING="${PROJECT_VERSION}")
set_target_properties(fsim PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
    OUTPUT_NAME fsim)
fsim_set_warnings(fsim)

# JSBSim.dll next to fsim.dll for consumers built in-tree.
add_custom_command(TARGET fsim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:fsim>)
