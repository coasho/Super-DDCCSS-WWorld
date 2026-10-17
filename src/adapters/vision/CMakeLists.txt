# fsim_vision.dll: vehicle cameras rendered offscreen (design 8.4) - the compatible World-handle API and its C ABI
# (ARCH-06: the adapter). Separate from fsim.dll because it needs Vulkan; links the public SDK for the world and
# vision_core (src/visualization/vision), which renders the frames this adapter reads from the world.
add_library(fsim_vision SHARED Vision.cpp fsim_vision_c.cpp)
add_library(fsim::vision ALIAS fsim_vision)
target_include_directories(fsim_vision
    PUBLIC  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_include_directories(fsim_vision SYSTEM PRIVATE ${CMAKE_SOURCE_DIR}/third_party/stb)
target_link_libraries(fsim_vision PUBLIC fsim::sdk PRIVATE fsim::vision_core fsim::ipc fsim::io fsim::core fsim::platform vsg::vsg vsgXchange::vsgXchange)
target_compile_definitions(fsim_vision PRIVATE FSIM_VISION_BUILDING)
set_target_properties(fsim_vision PROPERTIES CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON OUTPUT_NAME fsim_vision)
fsim_set_warnings(fsim_vision)
fsim_relax_third_party_warnings(fsim_vision)
