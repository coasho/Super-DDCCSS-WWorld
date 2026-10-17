# modeled_dms: modeled message delivery (ARCH-04) - RF-aware link delivery, queues, routing and provenance between
# platforms. Distinct from the generic network (simulation/communications/network), the OS transport and OMS wire
# formats. It propagates over MS RF paths; the generic network edge is declared as the design lays the layers out
# (docs/architecture/modules/simulation/plan.md, "Native services and communications").
add_library(fsim_modeled_dms STATIC
    Dms.h
    Dms.cpp)
add_library(fsim::modeled_dms ALIAS fsim_modeled_dms)
fsim_include_view(fsim_modeled_dms)
target_link_libraries(fsim_modeled_dms PUBLIC fsim::comm fsim::ms)
fsim_set_warnings(fsim_modeled_dms)
