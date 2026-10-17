# comm: abstract communication interfaces (design 9.6).
add_library(fsim_comm STATIC
    Comm.h
    Comm.cpp)
add_library(fsim::comm ALIAS fsim_comm)
fsim_include_view(fsim_comm)
target_link_libraries(fsim_comm PUBLIC fsim::control fsim::core fsim::platform)
fsim_set_warnings(fsim_comm)
