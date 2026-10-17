# vision_core: vehicle cameras rendered offscreen (ARCH-06; design 8.4) - the offscreen device and the camera scene
# built with the viewer's scene builders. It renders read-only vehicle frames; the World-handle API that gathers them
# is the vision adapter (src/adapters/vision, fsim_vision.dll), which links this library. Compiled as that library
# compiles its sources (FSIM_VISION_BUILDING, hidden visibility).
add_library(fsim_vision_core STATIC
    Offscreen.h
    Offscreen.cpp
    CameraScene.h
    CameraScene.cpp)
add_library(fsim::vision_core ALIAS fsim_vision_core)
fsim_include_view(fsim_vision_core)
target_link_libraries(fsim_vision_core
    PUBLIC  fsim::core vsg::vsg
    PRIVATE fsim::world fsim::io fsim::platform vsgXchange::vsgXchange)
target_compile_definitions(fsim_vision_core PRIVATE FSIM_VISION_BUILDING)
set_target_properties(fsim_vision_core PROPERTIES CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)
fsim_set_warnings(fsim_vision_core)
fsim_relax_third_party_warnings(fsim_vision_core)
