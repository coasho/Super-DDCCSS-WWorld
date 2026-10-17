# fsim._native: the Python SDK's native module (docs/sdk/python.md) - the one runtime type-registration owner for the
# package's native types, plain C over the C ABI, built against CPython's stable ABI (abi3) so one binary serves every
# interpreter from 3.11 on. ARCH-07 moved its sources here from python/src; python/CMakeLists.txt finds the interpreter,
# defines cpython::abi3 (its headers and python3.dll) and FSIM_PY_PACKAGE, adds this directory and stages the package.
#
# fsim_python_abi (abi/) is what fsim._native and fsim._vision share: fsim_py.h - the capsule both read and the busy
# flag of a call running without the GIL - and the stable-ABI build settings every extension module compiles with.
add_library(fsim_python_abi INTERFACE abi/fsim_py.h)
add_library(fsim::python_abi ALIAS fsim_python_abi)
target_include_directories(fsim_python_abi INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/abi)
target_link_libraries(fsim_python_abi INTERFACE cpython::abi3)
target_compile_definitions(fsim_python_abi INTERFACE Py_LIMITED_API=0x030B0000)
# C, not C++: the platform's C++ warning set does not apply, and CPython's slot tables need the function/object pointer
# casts -Wpedantic rejects.
target_compile_options(fsim_python_abi INTERFACE -Wall -Wextra -Wshadow -Wconversion -Wsign-conversion -Wcast-align
                       -Wnull-dereference $<$<BOOL:${FSIM_WARNINGS_AS_ERRORS}>:-Werror>)

add_library(fsim_python MODULE _native.c _native_dms.c _native_control.c _native_mission.c _native_world.c
    _native_scenario.c _native_environment.c _native_vecenv.c _native_recording.c _native_mission_env.c _native_world.h
    _native_world_options.h _native_vecenv.h _native_recording.h)
target_link_libraries(fsim_python PRIVATE fsim::sdk fsim::python_abi)
set_target_properties(fsim_python PROPERTIES
    PREFIX "" OUTPUT_NAME _native SUFFIX ".pyd"
    LIBRARY_OUTPUT_DIRECTORY ${FSIM_PY_PACKAGE}
    RUNTIME_OUTPUT_DIRECTORY ${FSIM_PY_PACKAGE}
    C_STANDARD 11 C_STANDARD_REQUIRED ON)
