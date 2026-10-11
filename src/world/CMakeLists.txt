# world: scene composition - Earth tiles, vehicle visuals, cameras.
add_library(fsim_world STATIC
    Earth.h
    Earth.cpp
    ElevatedTile.h
    ElevatedTile.cpp
    ElevationUpsampler.h
    ElevationUpsampler.cpp
    OfflineTiles.h
    OfflineTiles.cpp
    VehicleVisuals.h
    VehicleVisuals.cpp
    Exhaust.h
    Exhaust.cpp
    Airflow.h
    Airflow.cpp
    FlowVolumes.h
    FlowVolumes.cpp
    ShadingRate.h
    Blackbody.h
    Cocip.h
    Cocip.cpp
    WingVapour.h
    WingVapour.cpp
    Clouds.h
    Clouds.cpp
    CameraController.h
    Atmosphere.h
    Atmosphere.cpp
    CameraController.cpp
    Scattering.h
    Scattering.cpp
    Frames.h
    Interpolator.h
    Interpolator.cpp
    FlatGeometry.h
    FlatGeometry.cpp
    Sky.h
    Sky.cpp
    SkyDome.h
    SkyDome.cpp
    Trails.h
    Trails.cpp
    Terrain.h
    Terrain.cpp)
add_library(fsim::world ALIAS fsim_world)

target_include_directories(fsim_world PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(fsim_world PUBLIC fsim::sim fsim::render fsim::core vsg::vsg)
target_link_libraries(fsim_world PRIVATE fsim::io) # (PNG reading: the clouds' textures)
# NVIDIA Flow's solver (world::FlowSolver): a library of its own, the calibration tool's too (tools/flow_calibrate)
add_library(fsim_flow STATIC FlowSolver.h FlowSolver.cpp FlowFires.h FlowFires.cpp)
add_library(fsim::flow ALIAS fsim_flow)
target_include_directories(fsim_flow PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(fsim_flow PUBLIC fsim::core)
if(FSIM_HAS_FLOW)
    target_link_libraries(fsim_flow PRIVATE fsim_flow_headers)
    add_dependencies(fsim_flow fsim_flow_runtime)
endif()
fsim_set_warnings(fsim_flow)
target_link_libraries(fsim_world PUBLIC fsim::flow)
fsim_set_warnings(fsim_world)
fsim_relax_third_party_warnings(fsim_world)
