# ui: Dear ImGui layers drawn inside the VSG frame (design 9.5). They present read-only frames and the world stream's
# records (contracts); commands go through a separate adapter, never this library.
add_library(fsim_ui STATIC
    ViewerControls.h
    Colour.h
    MonitorGui.h
    MonitorGui.cpp
    KeyHandler.h
    KeyHandler.cpp
    CameraPreview.h
    CameraPreview.cpp
    MissionGui.h
    MissionGui.cpp)
add_library(fsim::ui ALIAS fsim_ui)

fsim_include_view(fsim_ui)
target_link_libraries(fsim_ui PUBLIC fsim::render fsim::world fsim::ipc fsim::core vsg::vsg vsgImGui::vsgImGui)
fsim_set_warnings(fsim_ui)
fsim_relax_third_party_warnings(fsim_ui)
