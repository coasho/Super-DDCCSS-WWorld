# session: the World behind the SDK - vehicles, stepping, environment,
# effects, comm and the shared-memory publisher (design 9.2).
add_library(fsim_session STATIC
    World.h
    World.cpp
    StoreFlightBinding.h
    StoreFlightBinding.cpp
    WorldDamage.cpp
    WorldEngagements.cpp
    AgentRules.h
    AgentRules.cpp
    Scenario.h
    Scenario.cpp
    Navigation.cpp
    PerformanceProfile.cpp
    StateData.cpp
    Dms.h
    Dms.cpp
    MaCore.h
    MaCore.cpp
    MaConsentLedger.h
    MaServiceBindings.h
    MaServiceBindings.cpp
    Frames.cpp
    Geometry.cpp
    Marshalls.cpp
    Intercepts.cpp)
add_library(fsim::session ALIAS fsim_session)
target_include_directories(fsim_session PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(fsim_session PUBLIC fsim::ms fsim::control fsim::effects fsim::comm fsim::ipc fsim::sim fsim::io fsim::core)
fsim_set_warnings(fsim_session)
