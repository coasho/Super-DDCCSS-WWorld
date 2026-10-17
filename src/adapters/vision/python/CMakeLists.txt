# fsim._vision: the optional camera module (docs/sdk/vision.md) over fsim_vision's C ABI, a separate extension module
# never registered in fsim._native. ARCH-07 moved its source here from python/src; python/CMakeLists.txt adds this
# directory when the renderer is built. It shares fsim_python_abi (src/adapters/sdk/python) with fsim._native.
add_library(fsim_python_vision MODULE _vision.c)
target_link_libraries(fsim_python_vision PRIVATE fsim::vision fsim::python_abi)
set_target_properties(fsim_python_vision PROPERTIES
    PREFIX "" OUTPUT_NAME _vision SUFFIX ".pyd"
    LIBRARY_OUTPUT_DIRECTORY ${FSIM_PY_PACKAGE}
    RUNTIME_OUTPUT_DIRECTORY ${FSIM_PY_PACKAGE}
    C_STANDARD 11 C_STANDARD_REQUIRED ON)
