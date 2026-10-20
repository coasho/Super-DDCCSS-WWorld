include(Catch)

# Exercise the user-facing entry points and a separately installed SDK. The
# consumer must link and run against the install tree, including runtime data.
if(WIN32)
    find_program(FSIM_POWERSHELL powershell REQUIRED)
    add_test(NAME launcher_contract
        COMMAND ${FSIM_POWERSHELL} -NoLogo -NoProfile -ExecutionPolicy Bypass
            -File ${CMAKE_SOURCE_DIR}/tests/compatibility/test_launchers.ps1
            -SourceRoot ${CMAKE_SOURCE_DIR})
    set_tests_properties(launcher_contract PROPERTIES LABELS launcher TIMEOUT 180)
endif()
add_test(NAME package_consumer
    COMMAND ${CMAKE_COMMAND}
        "-DSOURCE_DIR=${CMAKE_SOURCE_DIR}"
        "-DBUILD_DIR=${CMAKE_BINARY_DIR}"
        "-DTOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}"
        "-DGENERATOR=${CMAKE_GENERATOR}"
        "-DCONFIG=$<CONFIG>"
        "-DJSBSIM_DATA_DIR=${FSIM_JSBSIM_DATA_DIR}"
        "-DUCRT64_ROOT=${FSIM_UCRT64_ROOT}"
        "-DMAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}"
        -P ${CMAKE_SOURCE_DIR}/cmake/RunPackageConsumer.cmake)
set_tests_properties(package_consumer PROPERTIES LABELS package TIMEOUT 600)
add_test(NAME dist_components
    COMMAND ${CMAKE_COMMAND}
        "-DSOURCE_DIR=${CMAKE_SOURCE_DIR}"
        "-DTOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}"
        "-DUCRT64_ROOT=${FSIM_UCRT64_ROOT}"
        "-DGENERATOR=${CMAKE_GENERATOR}"
        "-DMAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}"
        -P ${CMAKE_SOURCE_DIR}/cmake/TestDistComponents.cmake)
set_tests_properties(dist_components PROPERTIES LABELS package TIMEOUT 120)
# The MA training interfaces through installed packages alone, in C++, C99 and Python (the Python package's
# interpreter is read from the build's own record at test time).
add_test(NAME mission_package_consumer
    COMMAND ${CMAKE_COMMAND}
        "-DSOURCE_DIR=${CMAKE_SOURCE_DIR}"
        "-DBUILD_DIR=${CMAKE_BINARY_DIR}"
        "-DTOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}"
        "-DGENERATOR=${CMAKE_GENERATOR}"
        "-DCONFIG=$<CONFIG>"
        "-DUCRT64_ROOT=${FSIM_UCRT64_ROOT}"
        "-DMAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}"
        -P ${CMAKE_SOURCE_DIR}/cmake/RunMissionPackageConsumer.cmake)
set_tests_properties(mission_package_consumer PROPERTIES LABELS "ma-training;package" TIMEOUT 600)

# Feature-owned equipment consumers use separate install trees from the generic package gates.
add_test(NAME equipment_package_consumer
    COMMAND ${CMAKE_COMMAND}
        "-DSOURCE_DIR=${CMAKE_SOURCE_DIR}"
        "-DBUILD_DIR=${CMAKE_BINARY_DIR}"
        "-DTOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}"
        "-DGENERATOR=${CMAKE_GENERATOR}"
        "-DCONFIG=$<CONFIG>"
        "-DJSBSIM_DATA_DIR=${FSIM_JSBSIM_DATA_DIR}"
        "-DUCRT64_ROOT=${FSIM_UCRT64_ROOT}"
        "-DMAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}"
        -P ${CMAKE_SOURCE_DIR}/cmake/RunEquipmentPackageConsumer.cmake)
set_tests_properties(equipment_package_consumer PROPERTIES LABELS "ma-training;package;native-engagement" TIMEOUT 600 RUN_SERIAL TRUE)

# Test executables are not part of any shipped package, so they are kept out of
# bin/ (which holds the applications) and out of the dist/ trees entirely. The
# runtime DLLs they need are staged beside them by the `deploy` target.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)

# Enumerate cases at test time rather than at build time: the executables are
# linked before `deploy` has staged the DLLs beside them, so running them to
# list their tests during the build would fail.
set(FSIM_CATCH_DISCOVERY DISCOVERY_MODE PRE_TEST)

foreach(core weapon_guidance fuze countermeasure_core)
    add_executable(fsim_${core}_test simulation/test_${core}.cpp)
    target_link_libraries(fsim_${core}_test PRIVATE fsim::ms)
    add_test(NAME ${core} COMMAND fsim_${core}_test)
endforeach()

add_executable(fsim_launch_eligibility_test simulation/test_launch_eligibility.cpp)
target_link_libraries(fsim_launch_eligibility_test PRIVATE fsim::ms)
fsim_set_warnings(fsim_launch_eligibility_test)
add_test(NAME launch_eligibility COMMAND fsim_launch_eligibility_test)

add_executable(fsim_engagement_native_test simulation/test_engagement_native.cpp)
target_link_libraries(fsim_engagement_native_test PRIVATE fsim::session)
target_compile_definitions(fsim_engagement_native_test PRIVATE FSIM_TEST_JSBSIM_ROOT="${FSIM_JSBSIM_DATA_DIR}")
add_test(NAME engagement_native COMMAND fsim_engagement_native_test)

add_executable(fsim_strike_capture_test visualization/test_strike_capture.cpp)
target_link_libraries(fsim_strike_capture_test PRIVATE fsim::ipc fsim::core)
# the viewer's applications/viewer/StrikeCapture.h: a view of the viewer's directory alone, built where the viewer is not
fsim_include_view(fsim_strike_capture_test DIRECTORIES ../src/applications/viewer)
fsim_set_warnings(fsim_strike_capture_test)
add_test(NAME strike_capture COMMAND fsim_strike_capture_test)
set_tests_properties(strike_capture PROPERTIES LABELS "showcase;native-engagement")

if(TARGET flightsim-showcase)
    add_executable(fsim_showcase_native_test integration/test_showcase_native.cpp)
    target_link_libraries(fsim_showcase_native_test PRIVATE fsim::ipc fsim::core fsim::sdk)
    fsim_set_warnings(fsim_showcase_native_test)
    add_test(NAME showcase_native COMMAND fsim_showcase_native_test
        $<TARGET_FILE:flightsim-showcase> ${CMAKE_BINARY_DIR}/showcase-native-test)
    set_tests_properties(showcase_native PROPERTIES LABELS "showcase;native-engagement"
        TIMEOUT 120 WORKING_DIRECTORY $<TARGET_FILE_DIR:flightsim-showcase>)
endif()

add_executable(fsim_tests
    simulation/test_units.cpp
    simulation/test_damage.cpp
    simulation/test_impact.cpp
    simulation/test_mass_change.cpp
    simulation/test_store_binding.cpp
    simulation/test_store_drag.cpp
    simulation/test_rng.cpp
    simulation/test_platform_timer.cpp
    simulation/test_registry.cpp
    simulation/test_vehicle_pool.cpp
    training/test_vecenv.cpp
    simulation/test_axes.cpp
    simulation/test_capabilities.cpp
    simulation/test_conformance.cpp
    simulation/test_ma_lifecycle.cpp
    simulation/test_control.cpp
    simulation/test_guidance.cpp
    simulation/test_interface.cpp
    simulation/test_modes.cpp
    simulation/test_load_limits.cpp
    simulation/test_routes.cpp
    simulation/test_patterns.cpp
    simulation/test_pattern_shapes.cpp
    simulation/test_curves.cpp
    simulation/test_nurbs.cpp
    simulation/test_curve_references.cpp
    simulation/test_route_points.cpp
    simulation/test_turn_points.cpp
    simulation/test_route_loiters.cpp
    simulation/test_route_segments.cpp
    simulation/test_route_arrivals.cpp
    simulation/test_route_states.cpp
    simulation/test_route_rnp.cpp
    simulation/test_route_paths.cpp
    simulation/test_route_branches.cpp
    simulation/test_route_terminators.cpp
    simulation/test_route_plans.cpp
    simulation/test_must_fly.cpp
    simulation/test_must_fly_zones.cpp
    simulation/test_must_fly_lines.cpp
    simulation/test_must_fly_volumes.cpp
    simulation/test_marshall.cpp
    simulation/test_launch.cpp
    simulation/test_taxi.cpp
    simulation/test_ground_route.cpp
    simulation/test_landing_route.cpp
    simulation/test_recovery.cpp
    simulation/test_drag_devices.cpp
    simulation/test_intercept.cpp
    simulation/test_grants.cpp
    simulation/test_mission_systems.cpp
    simulation/test_rf_path.cpp
    simulation/test_weapons.cpp
    simulation/test_stores.cpp
    simulation/test_rcs_table.cpp
    simulation/test_radar.cpp
    simulation/test_radar_modes.cpp
    simulation/test_esm.cpp
    simulation/test_rf_terrain_profile.cpp
    simulation/test_track.cpp
    simulation/test_signal.cpp
    simulation/test_control_compat.cpp
    simulation/test_profile.cpp
    simulation/test_applicability.cpp
    simulation/test_discovery.cpp
    simulation/test_behaviours.cpp
    simulation/test_fleet.cpp
    simulation/test_envelope.cpp
    simulation/test_schedule.cpp
    simulation/test_activities.cpp
    simulation/test_tasks.cpp
    simulation/test_reports.cpp
    simulation/test_tables.cpp
    simulation/test_navigation.cpp
    simulation/test_performance_profile.cpp
    simulation/test_protection.cpp
    simulation/test_laws.cpp
    simulation/test_manoeuvres.cpp
    simulation/test_rotorcraft.cpp
    simulation/test_support.cpp
    simulation/test_world.cpp
    simulation/test_terrain.cpp
    simulation/test_terrain_check.cpp
    simulation/test_terrain_strike.cpp
    simulation/test_altimeter.cpp
    simulation/test_state_data.cpp
    simulation/test_magnetic.cpp
    simulation/test_scenario.cpp
    simulation/test_comm.cpp)

target_link_libraries(fsim_tests PRIVATE fsim::env fsim::session fsim::ms fsim::control fsim::effects fsim::comm fsim::ipc fsim::sim fsim::io fsim::core Catch2::Catch2WithMain)
target_compile_definitions(fsim_tests PRIVATE FSIM_TEST_JSBSIM_ROOT="${FSIM_JSBSIM_DATA_DIR}"
                                                FSIM_TEST_AIRCRAFT_DIR="${CMAKE_SOURCE_DIR}/aircraft"
                                                FSIM_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/tests/simulation/data")
fsim_set_warnings(fsim_tests)

add_custom_command(TARGET fsim_tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:fsim_tests>)

catch_discover_tests(fsim_tests ${FSIM_CATCH_DISCOVERY} TEST_SPEC "~[conformance]~[fleet]")
# The fleet acceptance test (docs/flight-autonomy.md, 9.3): every capability every
# aircraft advertises, flown within its class's thresholds - labelled, so it runs
# on its own too (ctest -L fleet); CI runs it with the rest.
catch_discover_tests(fsim_tests ${FSIM_CATCH_DISCOVERY} TEST_SPEC "[fleet]" PROPERTIES LABELS fleet)
# Its longest case - every capability of every aircraft - in shards, each a process flying every n-th aircraft with a
# few unpinned workers, so ctest -j runs them side by side on every core (FSIM_FLEET_SHARD, simulation/test_fleet.cpp).
set(FSIM_FLEET_SHARDS 16 CACHE STRING "processes the fleet's capability case is split among")
set(FSIM_FLEET_SHARD_WORKERS 1 CACHE STRING "simulation threads each shard steps with")
math(EXPR _fsim_last_shard "${FSIM_FLEET_SHARDS} - 1")
foreach(_k RANGE ${_fsim_last_shard})
    set(_fsim_shard_name "fleet: every advertised capability flies its case within its class's thresholds - shard ${_k} of ${FSIM_FLEET_SHARDS}")
    add_test(NAME "${_fsim_shard_name}"
             COMMAND fsim_tests "fleet: every advertised capability flies its case within its class's thresholds")
    set_tests_properties("${_fsim_shard_name}" PROPERTIES LABELS fleet PROCESSORS ${FSIM_FLEET_SHARD_WORKERS} TIMEOUT 3600
        ENVIRONMENT "FSIM_FLEET_SHARD=${_k}/${FSIM_FLEET_SHARDS};FSIM_FLEET_WORKERS=${FSIM_FLEET_SHARD_WORKERS}")
endforeach()
# The control architecture's conformance (docs/control-architecture.md, step 6):
# the lifecycle's state machine for every adapter, the c172x checkpoints, the
# allocation gate and a short manoeuvre suite per adapter - labelled, so CI
# runs them as a step of their own (ctest -L conformance).
catch_discover_tests(fsim_tests ${FSIM_CATCH_DISCOVERY} TEST_SPEC "[conformance]" PROPERTIES LABELS conformance)

# The control architecture's measurements (docs/control-architecture.md,
# section 12): timings, heap allocations and trajectory digests. The
# allocation count is a test (none in the steady state); the rest is run by
# hand at each step.
add_executable(fsim_control_bench simulation/bench_control.cpp)
target_link_libraries(fsim_control_bench PRIVATE fsim::session fsim::control fsim::effects fsim::comm fsim::ipc fsim::sim fsim::io fsim::core)
target_compile_definitions(fsim_control_bench PRIVATE FSIM_TEST_JSBSIM_ROOT="${FSIM_JSBSIM_DATA_DIR}")
fsim_set_warnings(fsim_control_bench)
add_custom_command(TARGET fsim_control_bench POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:fsim_control_bench>)
add_test(NAME control_alloc COMMAND fsim_control_bench alloc)
set_tests_properties(control_alloc PROPERTIES LABELS conformance)

# The radar's cost and a digest of what it detected (tests/simulation/bench_radar.cpp), for A/B measurements of its code.
add_executable(fsim_radar_bench simulation/bench_radar.cpp)
target_link_libraries(fsim_radar_bench PRIVATE fsim::session fsim::ms fsim::control fsim::effects fsim::comm fsim::ipc fsim::sim fsim::io fsim::core)
target_compile_definitions(fsim_radar_bench PRIVATE FSIM_TEST_JSBSIM_ROOT="${FSIM_JSBSIM_DATA_DIR}")
fsim_set_warnings(fsim_radar_bench)
add_custom_command(TARGET fsim_radar_bench POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:fsim_radar_bench>)

# The public C++ SDK against fsim.dll (what a trainer links).
add_executable(fsim_sdk_tests compatibility/test_sdk.cpp compatibility/test_agents.cpp)
target_link_libraries(fsim_sdk_tests PRIVATE fsim::sdk Catch2::Catch2WithMain)
add_custom_command(TARGET fsim_sdk_tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:fsim> $<TARGET_FILE_DIR:fsim_sdk_tests>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:fsim_sdk_tests>)
target_compile_definitions(fsim_sdk_tests PRIVATE FSIM_TEST_JSBSIM_ROOT="${FSIM_JSBSIM_DATA_DIR}")
fsim_set_warnings(fsim_sdk_tests)
catch_discover_tests(fsim_sdk_tests ${FSIM_CATCH_DISCOVERY})

# Explicit MA-service binding through the public C++ SDK (docs/sdk/mission-training.md, section 9): configured
# ownership whatever order the agents were made in, refused overlaps and foreign controllers, the legacy path as it was.
add_executable(fsim_ma_service_binding_test training/test_ma_service_binding.cpp)
target_link_libraries(fsim_ma_service_binding_test PRIVATE fsim::sdk Catch2::Catch2WithMain)
fsim_set_warnings(fsim_ma_service_binding_test)
add_test(NAME ma_service_binding COMMAND fsim_ma_service_binding_test)
set_tests_properties(ma_service_binding PROPERTIES LABELS ma-training TIMEOUT 300)

# Legal reports with their provenance through the public C++ SDK (docs/sdk/mission-training.md, section 10): unknown,
# stale, unavailable and undelivered stay what they are, and hidden truth changes nothing an agent reads.
add_executable(fsim_ma_reports_test training/test_ma_reports.cpp)
target_link_libraries(fsim_ma_reports_test PRIVATE fsim::sdk Catch2::Catch2WithMain)
fsim_set_warnings(fsim_ma_reports_test)
add_test(NAME ma_reports COMMAND fsim_ma_reports_test)
set_tests_properties(ma_reports PROPERTIES LABELS ma-training TIMEOUT 300)

# The same contracts through the C ABI, from a plain C99 translation unit.
add_executable(fsim_ma_c_abi_test training/test_ma_c_abi.c)
target_link_libraries(fsim_ma_c_abi_test PRIVATE fsim::sdk)
set_target_properties(fsim_ma_c_abi_test PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON)
add_test(NAME ma_c_abi COMMAND fsim_ma_c_abi_test "${FSIM_JSBSIM_DATA_DIR}")
set_tests_properties(ma_c_abi PROPERTIES LABELS ma-training TIMEOUT 300)

# Same-build existing-native-state input for M4; this target does not implement episode ownership/reset.
add_library(fsim_native_state_reset_fixture STATIC
    training/fixtures/mission_training/NativeStateReset.cpp
    training/fixtures/mission_training/NativeStateResetCapture.cpp)
target_link_libraries(fsim_native_state_reset_fixture PUBLIC fsim::sdk fsim::session)
# (its header is spelled "fixtures/mission_training/NativeStateReset.h" by the tests of every category that use it)
target_include_directories(fsim_native_state_reset_fixture PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/training)
fsim_set_warnings(fsim_native_state_reset_fixture)
add_executable(fsim_native_state_reset_fixture_test training/test_native_state_reset_fixture.cpp)
target_link_libraries(fsim_native_state_reset_fixture_test PRIVATE fsim_native_state_reset_fixture)
target_link_libraries(fsim_native_state_reset_fixture_test PRIVATE fsim::sdk fsim::session Catch2::Catch2WithMain)
fsim_set_warnings(fsim_native_state_reset_fixture_test)
add_test(NAME native_state_reset_fixture COMMAND fsim_native_state_reset_fixture_test)
set_tests_properties(native_state_reset_fixture PROPERTIES LABELS "ma-training;native-engagement" TIMEOUT 300 RUN_SERIAL TRUE)

# Independent mission episodes (M4; docs/sdk/mission-training.md, sections 4 and 8): one World per episode, rebuilt
# whole on reset, generation-bound children, seed streams - and the supplied native-state fixture applied to the
# episode's own reset.
add_executable(fsim_mission_episode_test training/test_mission_episode.cpp)
target_link_libraries(fsim_mission_episode_test PRIVATE fsim::env fsim_native_state_reset_fixture fsim::sdk fsim::session fsim::core
                                                        Catch2::Catch2WithMain)
target_compile_definitions(fsim_mission_episode_test PRIVATE FSIM_TEST_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
                                                             FSIM_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/mission-episode-test")
if(WIN32)
    target_link_libraries(fsim_mission_episode_test PRIVATE psapi)
endif()
fsim_set_warnings(fsim_mission_episode_test)
add_test(NAME mission_episode COMMAND fsim_mission_episode_test)
set_tests_properties(mission_episode PROPERTIES LABELS "ma-training" TIMEOUT 900 RUN_SERIAL TRUE)

# The mission gateway through the public C++ SDK (M5 core; docs/sdk/mission-training.md, sections 3 to 14): requests,
# receipts and lifetime (ma_actions), snapshots, messages and referee documents (ma_observation). core::Json only reads
# the documents back.
foreach(name actions observation)
    add_executable(fsim_ma_${name}_test training/test_ma_${name}.cpp)
    target_link_libraries(fsim_ma_${name}_test PRIVATE fsim::sdk fsim::core Catch2::Catch2WithMain)
    target_compile_definitions(fsim_ma_${name}_test PRIVATE FSIM_TEST_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
                                                            FSIM_TEST_JSBSIM_ROOT="${FSIM_JSBSIM_DATA_DIR}")
    fsim_set_warnings(fsim_ma_${name}_test)
    add_test(NAME ma_${name} COMMAND fsim_ma_${name}_test)
    set_tests_properties(ma_${name} PROPERTIES LABELS ma-training TIMEOUT 600)
endforeach()

# Batches of mission episodes for training (M7 core; docs/sdk/mission-training.md, section 18): slots, roles, autoreset,
# final documents by principal, the referee's costs and actor views no hidden truth changes.
add_executable(fsim_mission_training_test training/test_mission_training.cpp)
target_link_libraries(fsim_mission_training_test PRIVATE fsim::sdk fsim::core Catch2::Catch2WithMain)
target_compile_definitions(fsim_mission_training_test PRIVATE FSIM_TEST_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
                                                              FSIM_TEST_JSBSIM_ROOT="${FSIM_JSBSIM_DATA_DIR}")
fsim_set_warnings(fsim_mission_training_test)
add_test(NAME mission_training COMMAND fsim_mission_training_test "~[concurrency]")
set_tests_properties(mission_training PROPERTIES LABELS ma-training TIMEOUT 900)
# Its contention cases (calls from other threads meeting batch transitions: hundreds of steps each) on their own, label
# stress, one CTest test each so ctest -j runs them side by side: the same cases, selected when batch, episode or
# binding concurrency code changes.
foreach(_stress_case "batch" "SameStep" "NextStep")
    if(_stress_case STREQUAL "batch")
        set(_stress_spec "[concurrency][batch]")
    else()
        set(_stress_spec "${_stress_case} under an actor's preopened calls*")
    endif()
    add_test(NAME "mission_training_stress ${_stress_case}" COMMAND fsim_mission_training_test "${_stress_spec}")
    # (two busy threads each - the range's and the actor's - which ctest -j counts; the cases' own deadline is a hang
    # guard under that timeout, not a speed bound)
    set_tests_properties("mission_training_stress ${_stress_case}" PROPERTIES LABELS "ma-training;stress" TIMEOUT 900 PROCESSORS 2)
endforeach()

# Joint acceptance, generic part (M8; docs/sdk/ma-training-validation.md): the recorded joint chains replayed through
# the public C++ SDK, and J2, J3 (messages), J4, J6, J7 and J8 (lifecycle) checked on what the endpoints returned.
add_executable(fsim_ma_joint_test training/test_ma_joint.cpp)
target_link_libraries(fsim_ma_joint_test PRIVATE fsim::sdk fsim::core Catch2::Catch2WithMain)
target_compile_definitions(fsim_ma_joint_test PRIVATE FSIM_TEST_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
                                                       FSIM_TEST_JSBSIM_ROOT="${FSIM_JSBSIM_DATA_DIR}")
fsim_set_warnings(fsim_ma_joint_test)
add_test(NAME ma_joint COMMAND fsim_ma_joint_test)
set_tests_properties(ma_joint PROPERTIES LABELS ma-training TIMEOUT 900)

# The same episodes through the C ABI, from a plain C99 translation unit: handle kinds, two-call reads, statuses.
add_executable(fsim_mission_c_abi_test training/test_mission_c_abi.c)
target_link_libraries(fsim_mission_c_abi_test PRIVATE fsim::sdk)
set_target_properties(fsim_mission_c_abi_test PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON)
add_test(NAME mission_c_abi COMMAND fsim_mission_c_abi_test "${CMAKE_SOURCE_DIR}/tests/training/fixtures/mission_training/contract"
                                    "${FSIM_JSBSIM_DATA_DIR}")
set_tests_properties(mission_c_abi PROPERTIES LABELS ma-training TIMEOUT 300)
foreach(_stress_mode same next)
    add_test(NAME "mission_c_abi_stress ${_stress_mode}" COMMAND fsim_mission_c_abi_test
             "${CMAKE_SOURCE_DIR}/tests/training/fixtures/mission_training/contract" "${FSIM_JSBSIM_DATA_DIR}" stress ${_stress_mode})
    set_tests_properties("mission_c_abi_stress ${_stress_mode}" PROPERTIES LABELS "ma-training;stress" TIMEOUT 900 PROCESSORS 2)
endforeach()

# The neutral contracts alone (ARCH-02): built against fsim::contracts and nothing else, so a contract that needed a
# domain or foundation library would not link here; it holds their frozen sizes, constants and defaults.
add_executable(fsim_contracts_test contracts/test_contracts.cpp)
target_link_libraries(fsim_contracts_test PRIVATE fsim::contracts Catch2::Catch2WithMain)
fsim_set_warnings(fsim_contracts_test)
add_test(NAME contracts COMMAND fsim_contracts_test)
set_tests_properties(contracts PROPERTIES LABELS architecture TIMEOUT 60)

# Include views (ARCH-03 task 5; cmake/IncludeViews.cmake): each migrated target compiles with its own headers and its
# declared providers' only. fsim_include_view_check(<module> <target> ALLOWED <headers> FORBIDDEN <headers>) compiles
# one-include probes with the real compiler in the target's own include environment (an interface library's: what a
# consumer of it alone gets): the allowed headers compile; each forbidden header - an unrelated private module's - is
# not found, spelled from src or from the repository root, and compiles once those roots are added back. No root may be
# src or an ancestor of it, however spelled (cmake/CheckIncludeView.cmake). No build is needed.
function(fsim_include_view_check module target)
    cmake_parse_arguments(_v "" "" "ALLOWED;FORBIDDEN" ${ARGN})
    get_target_property(_type ${target} TYPE)
    if(_type STREQUAL "INTERFACE_LIBRARY")
        set(_includes "$<TARGET_PROPERTY:${target},INTERFACE_INCLUDE_DIRECTORIES>")
        set(_defines "$<TARGET_PROPERTY:${target},INTERFACE_COMPILE_DEFINITIONS>")
        set(_options "$<TARGET_PROPERTY:${target},INTERFACE_COMPILE_OPTIONS>")
    else()
        set(_includes "$<TARGET_PROPERTY:${target},INCLUDE_DIRECTORIES>")
        set(_defines "$<TARGET_PROPERTY:${target},COMPILE_DEFINITIONS>")
        set(_options "$<TARGET_PROPERTY:${target},COMPILE_OPTIONS>")
    endif()
    if(CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
        set(_style MSVC)
    else()
        set(_style GNU)
    endif()
    set(_check "${CMAKE_BINARY_DIR}/include-view-checks/${target}.cmake")
    file(GENERATE OUTPUT "${_check}" CONTENT
"set(TARGET [==[${target}]==])
set(COMPILER [==[${CMAKE_CXX_COMPILER}]==])
set(STYLE ${_style})
set(SOURCE_DIR [==[${CMAKE_SOURCE_DIR}]==])
set(WORK [==[${CMAKE_BINARY_DIR}/include-view-checks/${target}]==])
set(INCLUDES [==[$<REMOVE_DUPLICATES:${_includes}>]==])
set(IMPLICIT [==[${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES}]==])
set(DEFINES [==[${_defines}]==])
set(OPTIONS [==[${_options}]==])
set(ALLOWED [==[${_v_ALLOWED}]==])
set(FORBIDDEN [==[${_v_FORBIDDEN}]==])
")
    add_test(NAME include_view_${module}
        COMMAND ${CMAKE_COMMAND} -DCHECK=${_check} -P ${CMAKE_SOURCE_DIR}/cmake/CheckIncludeView.cmake)
    set_tests_properties(include_view_${module} PROPERTIES LABELS architecture TIMEOUT 120)
endfunction()

fsim_include_view_check(contracts fsim_contracts
    ALLOWED   contracts/world_stream/WorldLayout.h contracts/simulation/WorldOptions.h
    FORBIDDEN foundation/core/Units.h simulation/flight/VehicleState.h)
fsim_include_view_check(platform fsim_platform
    ALLOWED   foundation/platform/Clock.h
    FORBIDDEN foundation/core/Units.h simulation/api/WorldOptions.h)
fsim_include_view_check(core fsim_core
    ALLOWED   foundation/core/Units.h foundation/platform/Clock.h
    FORBIDDEN foundation/io/AircraftFile.h simulation/flight/VehicleState.h)
fsim_include_view_check(io fsim_io
    ALLOWED   foundation/io/AircraftFile.h foundation/core/Units.h foundation/platform/Clock.h
    FORBIDDEN foundation/transport/WorldRegistry.h simulation/flight/VehicleState.h)
fsim_include_view_check(ipc fsim_ipc
    ALLOWED   foundation/transport/WorldRegistry.h contracts/world_stream/WorldLayout.h foundation/core/Units.h
              foundation/platform/Clock.h
    FORBIDDEN simulation/flight/VehicleState.h foundation/io/AircraftFile.h)
fsim_include_view_check(sim fsim_sim
    ALLOWED   simulation/flight/VehicleState.h contracts/world_stream/WorldLayout.h foundation/io/AircraftFile.h
              foundation/core/Units.h
    FORBIDDEN simulation/flight_control/Pid.h simulation/mission_systems/rf/Rain.h foundation/transport/WorldRegistry.h)
fsim_include_view_check(control fsim_control
    ALLOWED   simulation/flight_control/Pid.h simulation/flight/VehicleState.h foundation/core/Units.h
    FORBIDDEN simulation/environment/Effect.h simulation/mission_systems/rf/Rain.h foundation/transport/WorldRegistry.h)
fsim_include_view_check(effects fsim_effects
    ALLOWED   simulation/environment/Effect.h simulation/flight/VehicleState.h foundation/core/Units.h
    FORBIDDEN simulation/flight_control/Pid.h simulation/communications/network/Comm.h)
fsim_include_view_check(comm fsim_comm
    ALLOWED   simulation/communications/network/Comm.h simulation/flight_control/Pid.h foundation/platform/Clock.h
    FORBIDDEN simulation/mission_systems/rf/Rain.h simulation/communications/dms/Dms.h)
fsim_include_view_check(ms fsim_ms
    ALLOWED   simulation/mission_systems/rf/Rain.h simulation/mission_systems/radar/Radar.h foundation/core/Units.h
    FORBIDDEN simulation/flight/VehicleState.h simulation/flight_control/Pid.h
              simulation/runtime/services/MaServiceBindings.h)
# ARCH-04: modeled DMS sees the network and MS RF propagation, not the runtime above it; the runtime sees DMS and the
# leaves, not the facade, training or the SDK assembly; the facade sees the runtime and transport, not training or the
# SDK assembly.
fsim_include_view_check(modeled_dms fsim_modeled_dms
    ALLOWED   simulation/communications/dms/Dms.h simulation/mission_systems/rf/Path.h
              simulation/communications/network/Comm.h
    FORBIDDEN simulation/runtime/services/MaServiceBindings.h simulation/environment/Effect.h
              foundation/transport/WorldRegistry.h)
fsim_include_view_check(session fsim_session
    ALLOWED   simulation/runtime/world/World.h simulation/runtime/services/MaCore.h
              simulation/runtime/stores/StoreFlightBinding.h simulation/communications/dms/Dms.h
              foundation/transport/WorldRegistry.h
    FORBIDDEN simulation/api/WorldOptions.h training/native/vecenv/Registry.h adapters/sdk/c/LastError.h)
fsim_include_view_check(simulation_api fsim_simulation_api
    ALLOWED   simulation/api/WorldOptions.h simulation/runtime/world/World.h foundation/transport/Recording.h
    FORBIDDEN training/native/vecenv/Registry.h adapters/sdk/c/Handles.h)
# ARCH-05: training sees the simulation runtime and its lower facade, never the SDK assembly; the native episode
# owner and VecEnv (env) do not see the mission implementation above them.
fsim_include_view_check(env fsim_env
    ALLOWED   training/native/vecenv/Registry.h training/native/episodes/runtime/MissionEpisode.h
              simulation/api/WorldOptions.h simulation/runtime/world/World.h
    FORBIDDEN training/native/mission/MissionJson.h adapters/sdk/c/LastError.h)
fsim_include_view_check(mission fsim_training_mission
    ALLOWED   training/native/mission/MissionGateway.h training/native/episodes/api/MissionEpisodeImpl.h
              training/native/episodes/runtime/MissionEpisode.h simulation/api/WorldOptions.h
    FORBIDDEN adapters/sdk/c/Handles.h adapters/sdk/c/CAbiSupport.h adapters/sdk/c/LastError.h)
# ARCH-07: the SDK assembly compiles with its own private headers (adapters/sdk/c, spelled by name) and the views of
# the modules it links - no src root: its own headers by a src-root spelling, and an application's or the renderer's,
# are not found.
fsim_include_view_check(sdk fsim
    ALLOWED   training/native/vecenv/Registry.h training/native/episodes/runtime/MissionEpisode.h
              simulation/api/WorldOptions.h simulation/runtime/world/World.h
    FORBIDDEN adapters/sdk/c/Handles.h applications/demo/DemoSimulation.h visualization/scene/Interpolator.h)
# ARCH-06: the demo composes the flight module and sees neither the runtime, training nor the SDK assembly; the
# visualization targets (render builds) read contracts and read-only frames, never a simulation header. ARCH-08: the
# demo is its own publisher process (flightsim-demo), seeing the world stream's publisher and control block.
fsim_include_view_check(demo flightsim-demo
    ALLOWED   applications/demo/DemoSimulation.h simulation/flight/SimRunner.h foundation/transport/WorldPublisher.h
              contracts/world_stream/DemoControl.h
    FORBIDDEN simulation/runtime/services/MaServiceBindings.h training/native/vecenv/Registry.h adapters/sdk/c/LastError.h)
if(TARGET fsim_world)
    fsim_include_view_check(render fsim_render
        ALLOWED   visualization/render/Viewer.h foundation/core/Span.h
        FORBIDDEN visualization/scene/Frames.h simulation/flight/VehicleState.h)
    fsim_include_view_check(flow fsim_flow
        ALLOWED   visualization/vfx/flow/FlowSolver.h foundation/core/Span.h
        FORBIDDEN simulation/flight/VehicleState.h simulation/flight/SnapshotBuffer.h
                  applications/demo/DemoSimulation.h)
    fsim_include_view_check(scene fsim_world
        ALLOWED   visualization/scene/Frames.h visualization/vfx/effects/Trails.h visualization/vfx/flow/FlowSolver.h
                  visualization/render/Viewer.h contracts/world_stream/SnapshotBatch.h
                  foundation/transport/WorldMirror.h
        FORBIDDEN simulation/flight/VehicleState.h simulation/flight/SnapshotBuffer.h
                  simulation/runtime/services/MaServiceBindings.h visualization/ui/Colour.h)
    fsim_include_view_check(ui fsim_ui
        ALLOWED   visualization/ui/Colour.h visualization/scene/Frames.h foundation/transport/WorldMirror.h
                  contracts/world_stream/SnapshotBatch.h
        FORBIDDEN simulation/flight/SnapshotBuffer.h simulation/flight/VehicleState.h applications/demo/DemoSimulation.h
                  visualization/vision/CameraScene.h)
    fsim_include_view_check(vision_core fsim_vision_core
        ALLOWED   visualization/vision/CameraScene.h visualization/scene/Frames.h
        FORBIDDEN simulation/flight/VehicleState.h simulation/api/WorldOptions.h visualization/ui/Colour.h)
    # ARCH-08 (HC-16, HC-17): the viewer reads the world stream, its contracts and the demo control block; it sees no
    # demo simulation, flight model or control module - the control levels' names are the stream contract's.
    fsim_include_view_check(viewer flightsim-viewer
        ALLOWED   applications/viewer/DemoLink.h contracts/world_stream/DemoControl.h foundation/transport/WorldMirror.h
                  visualization/scene/Frames.h visualization/ui/Colour.h
        FORBIDDEN applications/demo/DemoSimulation.h simulation/flight_control/ControlStack.h simulation/flight/SimRunner.h
                  simulation/runtime/world/World.h)
    set_tests_properties(include_view_render include_view_flow include_view_scene include_view_ui include_view_vision_core
        include_view_viewer PROPERTIES LABELS "architecture;render")
endif()
# The check itself, on mutated copies of ms's check: the unmutated and a permitted root pass; src and its ancestors in
# each spelling (src/., the repository root, src/..), copied src trees, a provider's view removed, a stale probe and an
# unsupported compiler are refused (cmake/TestCheckIncludeView.cmake).
add_test(NAME include_view_checker
    COMMAND ${CMAKE_COMMAND} -DCHECK=${CMAKE_BINARY_DIR}/include-view-checks/fsim_ms.cmake
        -P ${CMAKE_SOURCE_DIR}/cmake/TestCheckIncludeView.cmake)
set_tests_properties(include_view_checker PROPERTIES LABELS architecture TIMEOUT 120)

# The lower simulation facade links on its own (ARCH-04): every object of fsim_simulation_api, linked whole into an
# executable with nothing but its own declared closure - the runtime, modeled DMS, the simulation leaves, transport and
# foundation - so a symbol it needed from training (fsim_env), the SDK assembly (fsim) or graphics would not resolve.
add_executable(fsim_simulation_api_link simulation/simulation_api_link.cpp)
target_link_libraries(fsim_simulation_api_link PRIVATE "$<LINK_LIBRARY:WHOLE_ARCHIVE,fsim::simulation_api>")
fsim_set_warnings(fsim_simulation_api_link)
add_test(NAME simulation_api_link COMMAND fsim_simulation_api_link)
set_tests_properties(simulation_api_link PROPERTIES LABELS "architecture;simulation" TIMEOUT 60)

# Training links on its own below the SDK assembly (ARCH-05): every object of fsim_training_mission and fsim_env, linked
# whole into a program with only their declared closure - the lower facade, the runtime and the simulation leaves - so
# a symbol they needed from the final SDK assembly (fsim), which links them, would not resolve: no
# training-to-SDK-to-training cycle.
add_executable(fsim_training_link training/training_link.cpp)
target_link_libraries(fsim_training_link PRIVATE "$<LINK_LIBRARY:WHOLE_ARCHIVE,fsim::mission,fsim::env>")
fsim_set_warnings(fsim_training_link)
add_test(NAME training_link COMMAND fsim_training_link)
set_tests_properties(training_link PROPERTIES LABELS "architecture;ma-training" TIMEOUT 60)

# Simulation construction, fixed state/transport ABI and output-only diagnostics stay independent of M5.
add_executable(fsim_simulation_interface_boundary_test
    simulation/test_episode_world_options.cpp
    contracts/test_state_layout.cpp
    simulation/test_simulation_output_boundary.cpp)
target_link_libraries(fsim_simulation_interface_boundary_test PRIVATE fsim::env fsim_native_state_reset_fixture
    fsim::sdk fsim::session fsim::ipc Catch2::Catch2WithMain)
target_compile_definitions(fsim_simulation_interface_boundary_test PRIVATE
    FSIM_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/simulation-interface-boundary-test")
fsim_set_warnings(fsim_simulation_interface_boundary_test)
add_test(NAME simulation_interface_boundary COMMAND fsim_simulation_interface_boundary_test)
set_tests_properties(simulation_interface_boundary PROPERTIES LABELS "architecture;simulation" TIMEOUT 300 RUN_SERIAL TRUE)

# Own-platform equipment access and report provenance through public C++ and C99 interfaces.
add_executable(fsim_ma_equipment_test training/test_ma_equipment.cpp)
target_link_libraries(fsim_ma_equipment_test PRIVATE fsim::sdk Catch2::Catch2WithMain)
fsim_set_warnings(fsim_ma_equipment_test)
add_test(NAME ma_equipment COMMAND fsim_ma_equipment_test)
set_tests_properties(ma_equipment PROPERTIES LABELS "ma-training;native-engagement" TIMEOUT 300)

add_executable(fsim_ma_equipment_c_test training/test_ma_equipment_c.c)
target_link_libraries(fsim_ma_equipment_c_test PRIVATE fsim::sdk)
set_target_properties(fsim_ma_equipment_c_test PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON)
add_test(NAME ma_equipment_c COMMAND fsim_ma_equipment_c_test "${FSIM_JSBSIM_DATA_DIR}")
set_tests_properties(ma_equipment_c PROPERTIES LABELS "ma-training;native-engagement" TIMEOUT 300)

# Public C control/Mission adapter behavior across the shared SDK boundary.
add_executable(fsim_adapter_compatibility_test compatibility/test_adapter_compatibility.cpp)
target_link_libraries(fsim_adapter_compatibility_test PRIVATE fsim::sdk)
fsim_set_warnings(fsim_adapter_compatibility_test)
add_test(NAME adapter_compatibility COMMAND fsim_adapter_compatibility_test)

# Public World, Scenario and Environment adapter behavior across the SDK boundary.
add_executable(fsim_world_adapter_compatibility_test compatibility/test_world_adapter_compatibility.cpp)
target_link_libraries(fsim_world_adapter_compatibility_test PRIVATE fsim::sdk)
fsim_set_warnings(fsim_world_adapter_compatibility_test)
add_test(NAME world_adapter_compatibility COMMAND fsim_world_adapter_compatibility_test)

# Public VecEnv and Recording adapter lifetimes and error behavior.
add_executable(fsim_batch_recording_adapter_compatibility_test compatibility/test_batch_recording_adapter_compatibility.cpp)
target_link_libraries(fsim_batch_recording_adapter_compatibility_test PRIVATE fsim::sdk)
fsim_set_warnings(fsim_batch_recording_adapter_compatibility_test)
add_test(NAME batch_recording_adapter_compatibility COMMAND fsim_batch_recording_adapter_compatibility_test "${FSIM_JSBSIM_DATA_DIR}")

# The public C ABI, consumed from a plain C translation unit against fsim.dll.
add_executable(fsim_c_abi_test compatibility/test_c_abi.c)
target_link_libraries(fsim_c_abi_test PRIVATE fsim::sdk)
set_target_properties(fsim_c_abi_test PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON)
add_test(NAME c_abi COMMAND fsim_c_abi_test "${FSIM_JSBSIM_DATA_DIR}")

# The crash handler writes a minidump (the crasher is expected to die).
add_executable(fsim_crash_test simulation/test_crash.cpp)
target_link_libraries(fsim_crash_test PRIVATE fsim::platform)
fsim_set_warnings(fsim_crash_test)
add_test(NAME crash_handler COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:fsim_crash_test> -DDIR=${CMAKE_BINARY_DIR}/crash-test -P ${CMAKE_SOURCE_DIR}/tests/simulation/crash_check.cmake)

# The Rust example against the C ABI (when cargo is installed): builds and steps a batch.
find_program(FSIM_CARGO cargo)
find_program(FSIM_RUSTC rustc)
set(FSIM_RUST_GNU OFF)
if(FSIM_CARGO AND FSIM_RUSTC)
    execute_process(COMMAND ${FSIM_RUSTC} -vV OUTPUT_VARIABLE _rustc_info ERROR_QUIET)
    if(_rustc_info MATCHES "host: x86_64-pc-windows-gnu")
        set(FSIM_RUST_GNU ON) # the DLLs are MinGW builds; the MSVC toolchain cannot link their import library
    endif()
endif()
if(FSIM_RUST_GNU)
    add_test(NAME rust_example
             COMMAND ${FSIM_CARGO} run --release --quiet -- vecenv
             WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/examples/training/rust_trainer)
    # FSIM_BUILD_DIR gives build.rs bin/ (the DLLs) and lib/ (the import library).
    set_tests_properties(rust_example PROPERTIES ENVIRONMENT "FSIM_BUILD_DIR=${CMAKE_BINARY_DIR}" TIMEOUT 600)
endif()

# Viewer-side terrain decoding (VSG data types; no Vulkan device needed).
if(FSIM_WITH_RENDER)
    add_executable(fsim_render_tests visualization/test_terrain_render.cpp visualization/test_vehicle_visuals.cpp visualization/test_cocip.cpp visualization/test_contrail_observed.cpp
                                     visualization/test_gpu_timer.cpp)
    target_link_libraries(fsim_render_tests PRIVATE fsim::world fsim::core Catch2::Catch2WithMain)
    target_compile_definitions(fsim_render_tests PRIVATE FSIM_TEST_AIRCRAFT_DIR="${CMAKE_SOURCE_DIR}/aircraft")
    fsim_set_warnings(fsim_render_tests)
    fsim_relax_third_party_warnings(fsim_render_tests)
    # (the render label: tools/validation/testing/scopes.toml selects these on ucrt64-release for a renderer change)
    catch_discover_tests(fsim_render_tests ${FSIM_CATCH_DISCOVERY} PROPERTIES LABELS render)
    # Fullscreen screenshot readback must capture the frame just presented (needs a Vulkan window).
    add_executable(fsim_viewer_screenshot_test visualization/test_viewer_screenshot.cpp)
    target_link_libraries(fsim_viewer_screenshot_test PRIVATE fsim::render vsgXchange::vsgXchange)
    fsim_set_warnings(fsim_viewer_screenshot_test)
    fsim_relax_third_party_warnings(fsim_viewer_screenshot_test)
    add_test(NAME viewer_screenshot COMMAND fsim_viewer_screenshot_test "${CMAKE_BINARY_DIR}/viewer-screenshot-test")
    set_tests_properties(viewer_screenshot PROPERTIES SKIP_RETURN_CODE 77 RUN_SERIAL TRUE TIMEOUT 60 LABELS render)
    # The RCS engine (ADR-30, MS-3; tools/models/rcs) against the closed forms: on a GPU with ray queries, else skipped.
    if(TARGET fsim_rcs_engine)
        add_executable(fsim_rcs_tests ${CMAKE_SOURCE_DIR}/tools/models/rcs/test_rcs.cpp)
        target_link_libraries(fsim_rcs_tests PRIVATE fsim_rcs_engine fsim::ms fsim::core Catch2::Catch2WithMain)
        target_compile_definitions(fsim_rcs_tests PRIVATE FSIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
        fsim_set_warnings(fsim_rcs_tests)
        add_test(NAME rcs COMMAND fsim_rcs_tests)
        set_tests_properties(rcs PROPERTIES SKIP_RETURN_CODE 4 LABELS render)
    endif()
endif()

# The demo publisher (ARCH-08, HC-16) flies the demo the viewer ran in-process, to the bit: the in-process
# DemoSimulation and flightsim-demo - launched and driven by the viewer's own client, DemoLink, through the demo control
# block - stepped together from the same paused start, every vehicle's state compared each step as the viewer reads it.
# Where the viewer is built (its client is the viewer's).
if(TARGET flightsim-viewer)
    add_executable(fsim_demo_publisher_test integration/test_demo_publisher.cpp
        ${CMAKE_SOURCE_DIR}/src/applications/demo/DemoSimulation.cpp
        ${CMAKE_SOURCE_DIR}/src/applications/viewer/DemoLink.cpp)
    target_include_directories(fsim_demo_publisher_test PRIVATE
        $<TARGET_PROPERTY:flightsim-demo,FSIM_INCLUDE_VIEW> $<TARGET_PROPERTY:flightsim-viewer,FSIM_INCLUDE_VIEW>)
    target_link_libraries(fsim_demo_publisher_test PRIVATE fsim::contracts fsim::ipc fsim::sim fsim::io fsim::core
        fsim::platform)
    fsim_set_warnings(fsim_demo_publisher_test)
    add_dependencies(fsim_demo_publisher_test flightsim-demo)
    add_test(NAME demo_publisher COMMAND fsim_demo_publisher_test $<TARGET_FILE:flightsim-demo> "${FSIM_JSBSIM_DATA_DIR}")
    # its control block's single steps counted - many requested at once are as many steps, those requested while it
    # runs and through a resume are taken at the next hold, it stops with thousands pending (HC-28, HC-30) - and
    # failed starts leaving no process handle (HC-29)
    add_test(NAME demo_publisher_steps COMMAND fsim_demo_publisher_test $<TARGET_FILE:flightsim-demo>
             "${FSIM_JSBSIM_DATA_DIR}" steps)
    add_test(NAME demo_publisher_lifecycle COMMAND fsim_demo_publisher_test $<TARGET_FILE:flightsim-demo>
             "${FSIM_JSBSIM_DATA_DIR}" lifecycle)
    set_tests_properties(demo_publisher demo_publisher_steps demo_publisher_lifecycle PROPERTIES LABELS render TIMEOUT 300)
endif()

# The demo publisher's single-step accounting (DemoSteps; HC-28, HC-30): against a model of the runner's thread in
# deterministic schedules - the delayed-work one, requests while running, a backlog through a resume and a pause, and
# seeded random ones - and on the real runner in-process. Wherever the publisher is built.
add_executable(fsim_demo_steps_test integration/test_demo_steps.cpp ${CMAKE_SOURCE_DIR}/src/applications/demo/DemoSimulation.cpp)
target_include_directories(fsim_demo_steps_test PRIVATE $<TARGET_PROPERTY:flightsim-demo,FSIM_INCLUDE_VIEW>)
target_link_libraries(fsim_demo_steps_test PRIVATE fsim::contracts fsim::ipc fsim::sim fsim::io fsim::core fsim::platform)
fsim_set_warnings(fsim_demo_steps_test)
add_test(NAME demo_publisher_schedules COMMAND fsim_demo_steps_test "${FSIM_JSBSIM_DATA_DIR}")
set_tests_properties(demo_publisher_schedules PROPERTIES TIMEOUT 300)

# Offscreen cameras through fsim_vision.dll (needs a Vulkan device).
if(FSIM_WITH_RENDER)
    add_executable(fsim_vision_test visualization/test_vision.cpp)
    target_link_libraries(fsim_vision_test PRIVATE fsim::sdk fsim::vision)
    add_custom_command(TARGET fsim_vision_test POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:fsim_vision> $<TARGET_FILE_DIR:fsim_vision_test>)
    fsim_set_warnings(fsim_vision_test)
    add_test(NAME vision COMMAND fsim_vision_test "${FSIM_JSBSIM_DATA_DIR}")
    set_tests_properties(vision PROPERTIES SKIP_RETURN_CODE 77 LABELS render) # no Vulkan device (CI runners)
    add_executable(fsim_vision_c_abi_test visualization/test_vision_c_abi.c)
    target_link_libraries(fsim_vision_c_abi_test PRIVATE fsim::sdk fsim::vision)
    set_target_properties(fsim_vision_c_abi_test PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON)
    add_test(NAME vision_c_abi COMMAND fsim_vision_c_abi_test "${FSIM_JSBSIM_DATA_DIR}")
    set_tests_properties(vision_c_abi PROPERTIES SKIP_RETURN_CODE 77 LABELS render)
endif()

# NVIDIA Flow's fires against the observed ones (tools/validation/flow_calibrate check: its 12 cases), where Flow is built - it
# needs the GPU, some 20 s
if(TARGET flightsim-flow-calibrate)
    add_test(NAME "NVIDIA Flow: an engagement's fires against the observed - CCPS's fireballs, the photographed detonations, API RP 521's jet flames - each within its tolerance" COMMAND flightsim-flow-calibrate check WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set_tests_properties("NVIDIA Flow: an engagement's fires against the observed - CCPS's fireballs, the photographed detonations, API RP 521's jet flames - each within its tolerance"
        PROPERTIES LABELS render)
endif()
