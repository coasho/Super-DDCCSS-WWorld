# A store's model, moving parts and effects shown on a scripted recording (ADR-30, MS-7e): headless, no VSG.
add_executable(flightsim-showcase main.cpp)
target_link_libraries(flightsim-showcase PRIVATE fsim::session fsim::ipc fsim::io fsim::core)
fsim_set_warnings(flightsim-showcase)
