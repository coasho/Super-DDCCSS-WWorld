# flow: NVIDIA Flow's solver (world::FlowSolver) and its fires - the optional GPU visual-effects target (ARCH-06),
# a library of its own, the calibration tool's too (tools/validation/flow_calibrate). Its runtime is loaded dynamically; without
# Flow the solver is inert. Physical effects are simulation's (simulation/environment), not this.
add_library(fsim_flow STATIC FlowSolver.h FlowSolver.cpp FlowFires.h FlowFires.cpp)
add_library(fsim::flow ALIAS fsim_flow)
fsim_include_view(fsim_flow)
target_link_libraries(fsim_flow PUBLIC fsim::core)
if(FSIM_HAS_FLOW)
    target_link_libraries(fsim_flow PRIVATE fsim_flow_headers)
    add_dependencies(fsim_flow fsim_flow_runtime)
endif()
fsim_set_warnings(fsim_flow)
