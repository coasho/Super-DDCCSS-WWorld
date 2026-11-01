add_executable(fsim-native-flight-probe main.cpp)
target_link_libraries(fsim-native-flight-probe PRIVATE fsim::session)
target_compile_definitions(fsim-native-flight-probe PRIVATE FSIM_PROBE_JSBSIM_ROOT="${FSIM_JSBSIM_DATA_DIR}")
fsim_set_warnings(fsim-native-flight-probe)
