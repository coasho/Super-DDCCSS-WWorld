# Example trainers are demo code against the public SDK. They are kept out of
# bin/ (the applications) and out of the SDK package, so that what a consumer
# gets and what merely demonstrates it are never the same directory.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples)

add_subdirectory(training/minimal_trainer)
add_subdirectory(simulation/multi_level_control)
add_subdirectory(training/ppo_trainer)
add_subdirectory(training/python/plugin)
add_subdirectory(simulation/scenario_runner)
add_subdirectory(simulation/udp_peer)
if(FSIM_WITH_RENDER)
    add_subdirectory(visualization/vision_capture)
endif()
