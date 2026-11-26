# fsim-dlz-generator (SIM-M04 / MS-8): native 6-DOF JSBSim flyout envelope
# generator producing deterministic .dlz-v1.json tables, propagated
# uncertainty bounds, and published-performance comparisons for all 24 AAMs.
add_executable(fsim-dlz-generator dlz_generator.cpp)
target_link_libraries(fsim-dlz-generator PRIVATE fsim::session fsim::ms fsim::sim fsim::io fsim::core)
fsim_set_warnings(fsim-dlz-generator)
