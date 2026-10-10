# NVIDIA Flow's fireballs measured against the observed ones (MS-7h): headless, the solver alone (world::FlowSolver).
add_executable(flightsim-flow-calibrate main.cpp)
target_link_libraries(flightsim-flow-calibrate PRIVATE fsim::flow fsim::core)
fsim_set_warnings(flightsim-flow-calibrate)
