# Module layering (design section 5). CMake declares the native target graph;
# tools/architecture checks source ownership and header imports against it,
# including optional renderer targets when the configured build is headless.
# Foundation (src/foundation) and the simulation's leaves (src/simulation) are in their ARCH-03 destinations; their
# target names and aliases are unchanged. Each exports only its include view (cmake/IncludeViews.cmake), not the src
# root. The simulation runtime (simulation/runtime, target fsim_session), modeled DMS and the lower simulation facade
# (simulation/api) followed in ARCH-04, and training's native implementation (training/native: targets fsim_env and
# fsim_training_mission) in ARCH-05. Visualization (visualization/: render, scene, vfx, ui, vision), the vision
# adapter (adapters/vision) and the viewer and demo applications (applications/) followed in ARCH-06, and the SDK
# assembly (adapters/sdk: target fsim) in ARCH-07, and the headless application (applications/headless) in ARCH-08.
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
add_subdirectory(simulation/communications/dms)
add_subdirectory(simulation/runtime)
add_subdirectory(simulation/api)
add_subdirectory(training/native)
add_subdirectory(adapters/sdk)
if(FSIM_WITH_RENDER)
    add_subdirectory(visualization/render)
    add_subdirectory(visualization/vfx/flow)
    add_subdirectory(visualization/scene)
    add_subdirectory(visualization/ui)
    add_subdirectory(visualization/vision)
    add_subdirectory(adapters/vision)
endif()
add_subdirectory(applications/headless)
add_subdirectory(applications/demo)
if(FSIM_WITH_RENDER)
    add_subdirectory(applications/viewer)
endif()
