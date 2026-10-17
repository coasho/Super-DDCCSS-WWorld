# render: window, viewer, render graph, ImGui integration point. VSG/Vulkan
# types live here and in the rest of visualization only (design rule 5.2).
add_library(fsim_render STATIC
    Viewer.h
    Viewer.cpp
    Screenshot.cpp
    GpuTimer.h
    GpuTimer.cpp)
add_library(fsim::render ALIAS fsim_render)

fsim_include_view(fsim_render)
target_link_libraries(fsim_render
    PUBLIC fsim::core vsg::vsg vsgXchange::vsgXchange vsgImGui::vsgImGui)
target_include_directories(fsim_render SYSTEM PRIVATE ${CMAKE_SOURCE_DIR}/third_party/stb)
fsim_set_warnings(fsim_render)
fsim_relax_third_party_warnings(fsim_render)
