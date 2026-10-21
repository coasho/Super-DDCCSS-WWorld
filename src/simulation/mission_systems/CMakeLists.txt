# ms: Mission Systems (ADR-30, docs/mission-systems.md) - an aircraft's payloads
# from their cards and its fit, their capabilities, states and commands.
add_library(fsim_ms STATIC
    Cards.h
    Cards.cpp
    components/Profile.cpp
    components/Bindings.cpp
    components/Providers.cpp
    components/providers/Seeker.cpp
    components/providers/Guidance.cpp
    components/providers/Autopilot.cpp
    components/providers/Allocator.cpp
    components/providers/Actuator.cpp
    components/providers/Propulsion.cpp
    components/providers/Aerodynamics.cpp
    components/providers/Fuze.cpp
    components/providers/Warhead.cpp
    Host.h
    Host.cpp
    HostStores.cpp
    Model.h
    Model.cpp
    Names.cpp
    weapon/Guidance.cpp
    weapon/Composition.cpp
    weapon/Seeker.cpp
    weapon/PlantAdapter.cpp
    weapon/FuzeAdapter.cpp
    weapon/adapters/NativeFuze.cpp
    weapon/adapters/SweptPathFuzeV1.cpp
    weapon/adapters/TurningCpaFuzeV1.cpp
    weapon/ContactFuze.h
    weapon/ContactFuze.cpp
    weapon/Autopilot.cpp
    weapon/LaunchEligibility.h
    weapon/LaunchEligibility.cpp
    countermeasure/Dispenser.cpp
    esm/Receiver.h
    esm/Receiver.cpp
    damage/Damage.h
    damage/Damage.cpp
    damage/WarheadAdapter.cpp
    damage/adapters/NativeWarhead.cpp
    damage/adapters/ZonedCaseWarheadV1.cpp
    damage/adapters/SideCaseWarheadV1.cpp
    damage/State.h
    damage/State.cpp
    damage/Debris.h
    damage/Debris.cpp
    damage/Impact.h
    damage/Impact.cpp
    rf/Antenna.h
    rf/Antenna.cpp
    rf/Ntia.h
    rf/NtiaItm.cpp
    rf/NtiaP528.cpp
    rf/Path.h
    rf/Path.cpp
    rf/Rain.h
    rf/Rain.cpp
    rcs/Table.h
    rcs/Table.cpp
    radar/Detection.h
    radar/Detection.cpp
    radar/Radar.h
    radar/Radar.cpp
    radar/Signal.cpp
    track/Linalg.h
    track/Models.h
    track/Models.cpp
    track/Filters.h
    track/Filters.cpp
    track/Assignment.h
    track/Assignment.cpp
    track/Tracker.h
    track/Tracker.cpp)
add_library(fsim::ms ALIAS fsim_ms)
fsim_include_view(fsim_ms)
target_include_directories(fsim_ms PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_compile_definitions(fsim_ms PRIVATE FSIM_BUILDING)
target_link_libraries(fsim_ms PUBLIC fsim::core)
fsim_set_warnings(fsim_ms)
# NTIA's reference implementations (third_party/ntia, public domain), compiled as they are: their own warnings off
set_source_files_properties(rf/NtiaItm.cpp rf/NtiaP528.cpp PROPERTIES
    INCLUDE_DIRECTORIES ${CMAKE_SOURCE_DIR}/third_party/ntia
    COMPILE_OPTIONS "-w")
