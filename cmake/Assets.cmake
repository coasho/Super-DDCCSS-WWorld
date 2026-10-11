# Runtime asset tree assembled under the build directory: <build>/share/flightsim.
# io::AssetResolver looks in <exe>/../share/flightsim, so bin/ + share/ form the
# same layout an installed package has (design 11.3 "Distribution").
set(FSIM_SHARE_DIR ${CMAKE_BINARY_DIR}/share/flightsim)
file(MAKE_DIRECTORY ${FSIM_SHARE_DIR}/models)

# Copy manifests / notices from the source tree.
# (a store's model is its design's, aircraft/<name>/<name>.glb: cmake/StageDesigns.cmake)
file(GLOB _fsim_model_manifests ${CMAKE_SOURCE_DIR}/assets/models/*.manifest ${CMAKE_SOURCE_DIR}/assets/models/*.glb ${CMAKE_SOURCE_DIR}/assets/models/*.rcs.bin
     ${CMAKE_SOURCE_DIR}/assets/models/NOTICE.md)
foreach(_m ${_fsim_model_manifests})
    configure_file(${_m} ${FSIM_SHARE_DIR}/models/ COPYONLY)
endforeach()
# A model the source tree no longer has is taken out of the build's copy: the viewer looks in models/ before a
# design's folder, so one left behind would stand in for the design's (the stores' were here until hangar built them).
file(GLOB _fsim_staged_models ${FSIM_SHARE_DIR}/models/*.manifest ${FSIM_SHARE_DIR}/models/*.glb ${FSIM_SHARE_DIR}/models/*.rcs.bin)
foreach(_m ${_fsim_staged_models})
    get_filename_component(_name ${_m} NAME)
    if(NOT _name STREQUAL "Cesium_Air.glb" AND NOT EXISTS ${CMAKE_SOURCE_DIR}/assets/models/${_name})
        file(REMOVE ${_m})
    endif()
endforeach()

# Sample aircraft model (Apache-2.0, CesiumGS/cesium sample data), fetched once.
set(_fsim_air ${FSIM_SHARE_DIR}/models/Cesium_Air.glb)
if(NOT EXISTS ${_fsim_air})
    message(STATUS "flightsim: downloading sample vehicle model Cesium_Air.glb")
    file(DOWNLOAD
        https://raw.githubusercontent.com/CesiumGS/cesium/main/Apps/SampleData/models/CesiumAir/Cesium_Air.glb
        ${_fsim_air}
        STATUS _fsim_dl TIMEOUT 60)
    list(GET _fsim_dl 0 _fsim_dl_code)
    if(NOT _fsim_dl_code EQUAL 0)
        message(WARNING "flightsim: sample model download failed (${_fsim_dl}); the viewer falls back to the placeholder")
        file(REMOVE ${_fsim_air})
    endif()
endif()

# The clouds' files (takram's three-geospatial shaders and textures, the precomputed atmosphere; assets/clouds),
# copied on each build so an edited shader is the one the viewer reads.
add_custom_target(fsim_cloud_assets ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different ${CMAKE_SOURCE_DIR}/assets/clouds ${FSIM_SHARE_DIR}/clouds
    COMMENT "flightsim: the clouds' files into share/flightsim/clouds")

# The payload cards (ADR-30, section 6; assets/payloads): one per real payload type, with its sources.
add_custom_target(fsim_payload_cards ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different ${CMAKE_SOURCE_DIR}/assets/payloads ${FSIM_SHARE_DIR}/payloads
    COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different ${CMAKE_SOURCE_DIR}/assets/stores ${FSIM_SHARE_DIR}/stores
    COMMENT "flightsim: the payload and store cards into share/flightsim/payloads and stores")
