# flightsim.exe: headless runs, benchmarks and (later) --serve / --viewer.
add_executable(flightsim
    main.cpp
    Options.h
    Options.cpp)

target_link_libraries(flightsim PRIVATE fsim::sim fsim::io fsim::core fsim::platform)
# Its own headers are spelled from src ("app/Options.h"); the application keeps the shared src root until it moves
# (the libraries it links export only their include views; cmake/IncludeViews.cmake).
target_include_directories(flightsim PRIVATE ${CMAKE_SOURCE_DIR}/src)
fsim_set_warnings(flightsim)

# JSBSim.dll must sit next to the executable.
add_custom_command(TARGET flightsim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:flightsim>
    COMMENT "Copy JSBSim shared library next to flightsim")

# flightsim-viewer.exe is the viewer application's (src/applications/viewer).
