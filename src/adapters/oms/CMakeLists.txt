# External OMS / A-GRA / UCI protocol adapter (SIM-M08 / MS-12).
# Strictly layered above the public SDK (fsim::sdk) and foundation JSON (fsim::core):
# no simulation, foundation, training or libfsim.dll target depends on fsim_oms.
add_library(fsim_oms STATIC
    OmsTransport.h
    OmsTransport.cpp
    OmsSchema.h
    OmsSchema.cpp
    OmsAdapter.h
    OmsAdapter.cpp)
add_library(fsim::oms ALIAS fsim_oms)

fsim_include_view(fsim_oms)
target_link_libraries(fsim_oms PUBLIC fsim::sdk PRIVATE fsim::core)
fsim_set_warnings(fsim_oms)
