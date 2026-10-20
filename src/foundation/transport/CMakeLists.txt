# ipc: shared-memory world segment - publisher (fsim.dll) and mirror (viewer) - and the recording writer/reader: the
# transport mechanisms. Their wire layouts are contracts (src/contracts); the transport depends on no simulation module.
add_library(fsim_ipc STATIC
    WorldPublisher.h
    WorldPublisher.cpp
    WorldMirror.h
    WorldMirror.cpp
    WorldRegistry.h
    WorldRegistry.cpp
    Recording.h
    Recording.cpp
    VisionSegment.h
    VisionSegment.cpp)
target_sources(fsim_ipc PRIVATE WeaponControlsSegment.h WeaponControlsSegment.cpp)
add_library(fsim::ipc ALIAS fsim_ipc)

fsim_include_view(fsim_ipc)
target_link_libraries(fsim_ipc PUBLIC fsim::contracts fsim::core fsim::platform)
fsim_set_warnings(fsim_ipc)
