# flightsim.exe: headless runs, benchmarks and (later) --serve / --viewer.
add_executable(flightsim
    main.cpp
    Options.h
    Options.cpp)

target_link_libraries(flightsim PRIVATE fsim::sim fsim::io fsim::core fsim::platform)
fsim_set_warnings(flightsim)

# JSBSim.dll must sit next to the executable.
add_custom_command(TARGET flightsim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:flightsim>
    COMMENT "Copy JSBSim shared library next to flightsim")

if(FSIM_WITH_RENDER)
    add_executable(flightsim-viewer
        viewer_main.cpp
        DemoAutopilot.h)
    target_link_libraries(flightsim-viewer PRIVATE fsim::ui fsim::world fsim::render fsim::ipc fsim::control fsim::sim fsim::io fsim::core fsim::platform)
    # A GUI application: no console window when double-clicked. When started
    # from a terminal it attaches to that console (platform::attachParentConsole).
    set_target_properties(flightsim-viewer PROPERTIES WIN32_EXECUTABLE TRUE)
    fsim_set_warnings(flightsim-viewer)
    fsim_relax_third_party_warnings(flightsim-viewer)
    add_custom_command(TARGET flightsim-viewer POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:flightsim-viewer>)
    # The build tree mirrors a package: bin/ beside config/, so <exe>/../config
    # finds the same file either way and there is nothing to keep in step.
    configure_file(${CMAKE_SOURCE_DIR}/assets/config/viewer.json ${CMAKE_BINARY_DIR}/config/viewer.json COPYONLY)
    # The build tree's own maps/ (bin/../maps, where the viewer, the physics and
    # the cameras read tiles): a junction to the source tree's assets/maps, so
    # the gigabytes fetch-maps.cmd put there are not copied.
    if(WIN32 AND NOT EXISTS ${CMAKE_BINARY_DIR}/maps)
        file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/assets/maps)
        file(TO_NATIVE_PATH ${CMAKE_BINARY_DIR}/maps _fsim_maps_link)
        file(TO_NATIVE_PATH ${CMAKE_SOURCE_DIR}/assets/maps _fsim_maps_target)
        execute_process(COMMAND cmd /c mklink /J "${_fsim_maps_link}" "${_fsim_maps_target}" OUTPUT_QUIET ERROR_QUIET)
    endif()
endif()
