# flightsim-demo: the demo publisher (ARCH-08, HC-16) - the viewer's built-in demo, a few native aircraft flown by a
# small autopilot on an internal simulation thread (DemoSimulation), in a process of its own. It publishes the demo on
# the read-only world stream like any world; `flightsim-viewer --demo` launches it and drives its simulation thread
# through the demo control block (contracts/world_stream/DemoControl.h). Demo composition only: it composes the native
# flight module directly, as the demo always has; the live/replay viewer draws its snapshots and owns no flight model.
# Headless: it needs no graphics.
add_executable(flightsim-demo
    main.cpp
    DemoSimulation.h
    DemoSimulation.cpp
    DemoAutopilot.h)
fsim_include_view(flightsim-demo)
target_link_libraries(flightsim-demo PRIVATE fsim::contracts fsim::ipc fsim::sim fsim::io fsim::core fsim::platform)
fsim_set_warnings(flightsim-demo)
# JSBSim.dll must sit next to the executable.
add_custom_command(TARGET flightsim-demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:flightsim-demo>)
