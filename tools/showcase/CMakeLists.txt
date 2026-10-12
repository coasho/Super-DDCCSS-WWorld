# Native SDK engagement recordings, optionally published live; no VSG or client weapon physics.
add_executable(flightsim-showcase main.cpp)
target_link_libraries(flightsim-showcase PRIVATE fsim::session fsim::ipc fsim::io fsim::core)
fsim_set_warnings(flightsim-showcase)
