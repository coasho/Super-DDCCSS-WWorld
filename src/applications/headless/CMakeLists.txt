# flightsim.exe: headless runs, benchmarks and (later) --serve / --viewer (ARCH-08: the headless application,
# src/applications/headless).
add_executable(flightsim
    main.cpp
    Options.h
    Options.cpp)

# Its own header is spelled from src ("applications/headless/Options.h") through its include view; the libraries it
# links export only theirs (cmake/IncludeViews.cmake), so it sees no other module's private headers.
fsim_include_view(flightsim)
target_link_libraries(flightsim PRIVATE fsim::sim fsim::io fsim::core fsim::platform)
fsim_set_warnings(flightsim)

# JSBSim.dll must sit next to the executable.
add_custom_command(TARGET flightsim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:flightsim>
    COMMENT "Copy JSBSim shared library next to flightsim")

# flightsim-viewer.exe is the viewer application's (src/applications/viewer).
