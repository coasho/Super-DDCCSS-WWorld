# world: the presentation scene (ARCH-06; visualization/scene, target name kept) - Earth tiles, sky, clouds, vehicle
# visuals, cameras and presentation interpolation - and the visual effects drawn in it (visualization/vfx/effects:
# exhaust, airflow, vapour, trails, contrails and Flow's volumes). It reads read-only frames and contracts (vehicle
# states, snapshot batches, the world stream); it never steps or reads the simulation's implementation.
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
    ../vfx/effects/Exhaust.h
    ../vfx/effects/Exhaust.cpp
    ../vfx/effects/Airflow.h
    ../vfx/effects/Airflow.cpp
    ../vfx/effects/FlowVolumes.h
    ../vfx/effects/FlowVolumes.cpp
    ShadingRate.h
    Blackbody.h
    ../vfx/effects/Cocip.h
    ../vfx/effects/Cocip.cpp
    ../vfx/effects/WingVapour.h
    ../vfx/effects/WingVapour.cpp
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
    ../vfx/effects/Trails.h
    ../vfx/effects/Trails.cpp
    Terrain.h
    Terrain.cpp)
add_library(fsim::world ALIAS fsim_world)

fsim_include_view(fsim_world DIRECTORIES . ../vfx/effects)
target_link_libraries(fsim_world PUBLIC fsim::render fsim::core fsim::ipc vsg::vsg)
target_link_libraries(fsim_world PRIVATE fsim::io) # (PNG reading: the clouds' textures)
# NVIDIA Flow's solver, the optional GPU target, is visualization/vfx/flow's (fsim_flow).
target_link_libraries(fsim_world PUBLIC fsim::flow)
fsim_set_warnings(fsim_world)
fsim_relax_third_party_warnings(fsim_world)
