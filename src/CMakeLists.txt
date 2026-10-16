# Module layering (design section 5). CMake declares the native target graph;
# tools/architecture checks source ownership and header imports against it,
# including optional renderer targets when the configured build is headless.
# Foundation (src/foundation) and the simulation's leaves (src/simulation) are in their ARCH-03 destinations; their
# target names and aliases are unchanged. The runtime, training, visualization and assemblies move in later packages.
add_subdirectory(contracts)
add_subdirectory(foundation/platform)
add_subdirectory(foundation/core)
add_subdirectory(foundation/io)
add_subdirectory(simulation/flight)
add_subdirectory(foundation/transport)
add_subdirectory(simulation/flight_control)
add_subdirectory(simulation/environment)
add_subdirectory(simulation/communications/network)
add_subdirectory(simulation/mission_systems)
add_subdirectory(session)
add_subdirectory(env)
add_subdirectory(sdk)
if(FSIM_WITH_RENDER)
    add_subdirectory(render)
    add_subdirectory(world)
    add_subdirectory(ui)
    add_subdirectory(vision)
endif()
add_subdirectory(app)
