# NVIDIA Flow 2 (BSD-3-Clause; NVIDIA-Omniverse/PhysX, its flow/ folder): the viewer's fire and smoke solver
# (world::FlowVolumes, MS-7h). Optional: FSIM_FLOW_DIR names a Flow checkout built by tools/flow/build-flow.sh (the
# pinned commit is there). Flow is used through its C interface only - its loader opens nvflow.dll and nvflowext.dll
# at run time, so nothing of it links and the MinGW build needs no MSVC import library.
set(FSIM_FLOW_DIR "" CACHE PATH "A built NVIDIA Flow checkout (its flow/ folder): the viewer's fire and smoke solver")

set(FSIM_HAS_FLOW OFF)
if(FSIM_FLOW_DIR)
    set(_fsim_flow_bin ${FSIM_FLOW_DIR}/_build/windows-x86_64/release)
    if(EXISTS ${FSIM_FLOW_DIR}/include/nvflowext/NvFlowLoader.h AND EXISTS ${_fsim_flow_bin}/nvflow.dll
       AND EXISTS ${_fsim_flow_bin}/nvflowext.dll)
        set(FSIM_HAS_FLOW ON)
        add_library(fsim_flow_headers INTERFACE)
        target_include_directories(fsim_flow_headers SYSTEM INTERFACE
            ${FSIM_FLOW_DIR}/include/nvflow ${FSIM_FLOW_DIR}/include/nvflowext ${FSIM_FLOW_DIR}/include/nvflow/nanovdb)
        target_compile_definitions(fsim_flow_headers INTERFACE FSIM_HAS_FLOW=1)
        # its two libraries beside the executables (the loader opens them by name)
        add_custom_target(fsim_flow_runtime ALL
            COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/bin
            COMMAND ${CMAKE_COMMAND} -E copy_if_different ${_fsim_flow_bin}/nvflow.dll ${_fsim_flow_bin}/nvflowext.dll ${CMAKE_BINARY_DIR}/bin
            COMMENT "flightsim: NVIDIA Flow's libraries into bin/")
        message(STATUS "flightsim: NVIDIA Flow from ${FSIM_FLOW_DIR}")
    else()
        message(WARNING "flightsim: FSIM_FLOW_DIR=${FSIM_FLOW_DIR} has no built Flow (run tools/flow/build-flow.sh); built without it")
    endif()
endif()
