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
target_sources(fsim_ipc PRIVATE FusionSegment.h FusionSegment.cpp)
target_sources(fsim_ipc PRIVATE WeaponDiagnosticsV1.h WeaponDiagnosticsV1.cpp)
target_sources(fsim_ipc PRIVATE WeaponDiagnosticCaptureBindingV1.h WeaponDiagnosticCaptureBindingV1.cpp)
target_include_directories(fsim_ipc SYSTEM PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/picosha2")
# The statically linked SDK and Viewer both redistribute the MIT implementation.
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/third_party/picosha2/LICENSE"
    DESTINATION share/flightsim/licenses RENAME PicoSHA2-LICENSE.txt COMPONENT sdk)
if(FSIM_WITH_RENDER)
    install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/third_party/picosha2/LICENSE"
        DESTINATION share/flightsim/licenses RENAME PicoSHA2-LICENSE.txt COMPONENT viewer)
endif()
target_sources(fsim_ipc PRIVATE WeaponThermalBandV1.h WeaponThermalBandV1.cpp)
add_library(fsim::ipc ALIAS fsim_ipc)

fsim_include_view(fsim_ipc)
target_link_libraries(fsim_ipc PUBLIC fsim::contracts fsim::core fsim::platform)
fsim_set_warnings(fsim_ipc)
