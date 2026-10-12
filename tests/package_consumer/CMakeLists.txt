# Standalone consumer: configure against one installed SDK prefix only.
cmake_minimum_required(VERSION 3.25)
project(fsim_consumer CXX)
find_package(fsim CONFIG REQUIRED COMPONENTS sdk)
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE fsim::sdk)
target_compile_features(consumer PRIVATE cxx_std_17)
set_target_properties(consumer PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${fsim_BIN_DIR}")
foreach(_config Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${_config}" _upper)
    set_target_properties(consumer PROPERTIES RUNTIME_OUTPUT_DIRECTORY_${_upper} "${fsim_BIN_DIR}")
endforeach()
if(TARGET fsim::vision)
    # Check package availability without opening a GPU device.
    target_compile_definitions(consumer PRIVATE HAVE_FSIM_VISION)
endif()
add_custom_target(run
    COMMAND ${CMAKE_COMMAND} -E env
        "FSIM_AIRCRAFT_PATH=${fsim_AIRCRAFT_DIR};${fsim_AIRCRAFT_DIR}/stores"
        $<TARGET_FILE:consumer> "${fsim_BIN_DIR}/.." smoke
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}" DEPENDS consumer VERBATIM)
