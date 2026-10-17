# platform: the only module allowed to include OS headers (design rule 5.2).
add_library(fsim_platform STATIC
    Clock.h
    Memory.h
    Paths.h
    Threads.h
    SharedMemory.h
    SharedMemory.cpp
    Http.h
    Http.cpp
    Udp.h
    Udp.cpp
    CrashHandler.h
    CrashHandler.cpp
    Platform.cpp)
add_library(fsim::platform ALIAS fsim_platform)

fsim_include_view(fsim_platform)
# (the map set when nothing beside the program has one: a development build's SDK loaded from its staged Python copy)
target_compile_definitions(fsim_platform PRIVATE FSIM_SOURCE_MAPS_DIR="${CMAKE_SOURCE_DIR}/assets/maps")
if(WIN32)
    target_link_libraries(fsim_platform PUBLIC winmm winhttp ws2_32)
endif()
fsim_set_warnings(fsim_platform)
