# flightsim-rcs (ADR-30, MS-3): radar cross section tables by shooting and
# bouncing rays on the GPU (Vulkan ray queries, through VSG).
add_library(fsim_rcs_engine STATIC Engine.h Engine.cpp Model.h Model.cpp Shapes.h Shapes.cpp)
target_include_directories(fsim_rcs_engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(fsim_rcs_engine PUBLIC fsim::core PRIVATE vsg::vsg)
fsim_set_warnings(fsim_rcs_engine)
fsim_relax_third_party_warnings(fsim_rcs_engine)

add_executable(flightsim-rcs main.cpp)
target_link_libraries(flightsim-rcs PRIVATE fsim_rcs_engine fsim::ms fsim::core)
fsim_set_warnings(flightsim-rcs)

# (its tests: tests/CMakeLists.txt, fsim_rcs_tests)
