# NVIDIA Flow 2 (BSD-3-Clause; NVIDIA-Omniverse/PhysX, its flow/ folder): the viewer's fire and smoke solver
# (world::FlowVolumes, MS-7h) - the one way an engagement's detonations and fires are drawn, so a build with the
# viewer has it or does not configure.
#
# Its headers and its two libraries are in third_party/flow (2.2.0, the commit tools/flow/build-flow.sh pins; built
# with MSVC by that script), so a clone builds with it and needs neither MSVC nor NVIDIA's packman. FSIM_FLOW_DIR
# names another checkout built by that script instead (its flow/ folder). Flow is used through its C interface only -
# its loader opens nvflow.dll and nvflowext.dll at run time, so nothing of it links and the MinGW build needs no MSVC
# import library.
set(FSIM_FLOW_DIR "" CACHE PATH "A built NVIDIA Flow checkout (its flow/ folder) instead of third_party/flow")

set(FSIM_HAS_FLOW OFF)
if(NOT FSIM_WITH_RENDER)
    return() # (a headless build draws nothing: no solver, no calibration against the GPU)
endif()
if(FSIM_FLOW_DIR)
    set(_fsim_flow_include ${FSIM_FLOW_DIR}/include)
    set(_fsim_flow_bin ${FSIM_FLOW_DIR}/_build/windows-x86_64/release)
else()
    set(_fsim_flow_include ${CMAKE_SOURCE_DIR}/third_party/flow/include)
    set(_fsim_flow_bin ${CMAKE_SOURCE_DIR}/third_party/flow/bin/windows-x86_64)
endif()
if(EXISTS ${_fsim_flow_include}/nvflowext/NvFlowLoader.h AND EXISTS ${_fsim_flow_bin}/nvflow.dll
   AND EXISTS ${_fsim_flow_bin}/nvflowext.dll)
    set(FSIM_HAS_FLOW ON)
    set(FSIM_FLOW_RUNTIME ${_fsim_flow_bin}/nvflow.dll ${_fsim_flow_bin}/nvflowext.dll)
    add_library(fsim_flow_headers INTERFACE)
    target_include_directories(fsim_flow_headers SYSTEM INTERFACE
        ${_fsim_flow_include}/nvflow ${_fsim_flow_include}/nvflowext ${_fsim_flow_include}/nvflow/nanovdb)
    target_compile_definitions(fsim_flow_headers INTERFACE FSIM_HAS_FLOW=1)
    # its two libraries beside the executables (the loader opens them by name)
    add_custom_target(fsim_flow_runtime ALL
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/bin
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FSIM_FLOW_RUNTIME} ${CMAKE_BINARY_DIR}/bin
        COMMENT "flightsim: NVIDIA Flow's libraries into bin/")
    message(STATUS "flightsim: NVIDIA Flow from ${_fsim_flow_bin}")
else()
    message(FATAL_ERROR "flightsim: no NVIDIA Flow at ${_fsim_flow_bin} (its headers at ${_fsim_flow_include}). "
        "The viewer draws an engagement's fire with it and with nothing else: restore third_party/flow, or build a "
        "checkout with tools/flow/build-flow.sh and configure with -DFSIM_FLOW_DIR=<checkout>/flow.")
endif()
