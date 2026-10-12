include(Catch)

# Exercise the user-facing entry points and a separately installed SDK. The
# consumer must link and run against the install tree, including runtime data.
if(WIN32)
    find_program(FSIM_POWERSHELL powershell REQUIRED)
    add_test(NAME launcher_contract
        COMMAND ${FSIM_POWERSHELL} -NoLogo -NoProfile -ExecutionPolicy Bypass
            -File ${CMAKE_SOURCE_DIR}/tests/test_launchers.ps1
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

# Test executables are not part of any shipped package, so they are kept out of
# bin/ (which holds the applications) and out of the dist/ trees entirely. The
# runtime DLLs they need are staged beside them by the `deploy` target.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests)

# Enumerate cases at test time rather than at build time: the executables are
# linked before `deploy` has staged the DLLs beside them, so running them to
# list their tests during the build would fail.
set(FSIM_CATCH_DISCOVERY DISCOVERY_MODE PRE_TEST)

foreach(core weapon_guidance countermeasure_core)
    add_executable(fsim_${core}_test test_${core}.cpp)
    target_link_libraries(fsim_${core}_test PRIVATE fsim::ms)
    add_test(NAME ${core} COMMAND fsim_${core}_test)
endforeach()

add_executable(fsim_launch_eligibility_test test_launch_eligibility.cpp)
target_link_libraries(fsim_launch_eligibility_test PRIVATE fsim::ms)
fsim_set_warnings(fsim_launch_eligibility_test)
add_test(NAME launch_eligibility COMMAND fsim_launch_eligibility_test)

add_executable(fsim_engagement_native_test test_engagement_native.cpp)
target_link_libraries(fsim_engagement_native_test PRIVATE fsim::session)
target_compile_definitions(fsim_engagement_native_test PRIVATE FSIM_TEST_JSBSIM_ROOT="${FSIM_JSBSIM_DATA_DIR}")
add_test(NAME engagement_native COMMAND fsim_engagement_native_test)

add_executable(fsim_tests
    test_units.cpp
    test_damage.cpp
    test_impact.cpp
    test_mass_change.cpp
    test_store_drag.cpp
    test_rng.cpp
    test_registry.cpp
    test_vehicle_pool.cpp
    test_vecenv.cpp
    test_axes.cpp
    test_capabilities.cpp
    test_conformance.cpp
    test_control.cpp
    test_guidance.cpp
    test_interface.cpp
    test_modes.cpp
    test_load_limits.cpp
    test_routes.cpp
    test_patterns.cpp
    test_pattern_shapes.cpp
    test_curves.cpp
    test_nurbs.cpp
    test_curve_references.cpp
    test_route_points.cpp
    test_turn_points.cpp
    test_route_loiters.cpp
    test_route_segments.cpp
    test_route_arrivals.cpp
    test_route_states.cpp
    test_route_rnp.cpp
    test_route_paths.cpp
    test_route_branches.cpp
    test_route_terminators.cpp
    test_route_plans.cpp
    test_must_fly.cpp
    test_must_fly_zones.cpp
    test_must_fly_lines.cpp
    test_must_fly_volumes.cpp
    test_marshall.cpp
    test_launch.cpp
    test_taxi.cpp
    test_ground_route.cpp
    test_landing_route.cpp
    test_recovery.cpp
    test_drag_devices.cpp
    test_intercept.cpp
    test_grants.cpp
    test_mission_systems.cpp
    test_rf_path.cpp
    test_weapons.cpp
    test_stores.cpp
    test_rcs_table.cpp
    test_radar.cpp
    test_radar_modes.cpp
    test_esm.cpp
    test_rf_terrain_profile.cpp
    test_track.cpp
    test_signal.cpp
    test_control_compat.cpp
    test_profile.cpp
    test_applicability.cpp
    test_discovery.cpp
    test_behaviours.cpp
    test_fleet.cpp
    test_envelope.cpp
    test_schedule.cpp
    test_activities.cpp
    test_tasks.cpp
    test_reports.cpp
    test_tables.cpp
    test_navigation.cpp
    test_performance_profile.cpp
    test_protection.cpp
    test_laws.cpp
    test_manoeuvres.cpp
    test_rotorcraft.cpp
    test_support.cpp
    test_world.cpp
    test_terrain.cpp
    test_terrain_check.cpp
    test_terrain_strike.cpp
    test_altimeter.cpp
    test_state_data.cpp
    test_magnetic.cpp
    test_scenario.cpp
    test_comm.cpp)

target_link_libraries(fsim_tests PRIVATE fsim::env fsim::session fsim::ms fsim::control fsim::effects fsim::comm fsim::ipc fsim::sim fsim::io fsim::core Catch2::Catch2WithMain)
target_compile_definitions(fsim_tests PRIVATE FSIM_TEST_JSBSIM_ROOT="${FSIM_JSBSIM_DATA_DIR}"
                                                FSIM_TEST_AIRCRAFT_DIR="${CMAKE_SOURCE_DIR}/aircraft"
                                                FSIM_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/tests/data")
fsim_set_warnings(fsim_tests)

add_custom_command(TARGET fsim_tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:fsim_tests>)

catch_discover_tests(fsim_tests ${FSIM_CATCH_DISCOVERY} TEST_SPEC "~[conformance]~[fleet]")
# The fleet acceptance test (docs/flight-autonomy.md, 9.3): every capability every
# aircraft advertises, flown within its class's thresholds - labelled, so it runs
# on its own too (ctest -L fleet); CI runs it with the rest.
catch_discover_tests(fsim_tests ${FSIM_CATCH_DISCOVERY} TEST_SPEC "[fleet]" PROPERTIES LABELS fleet)
# Its longest case - every capability of every aircraft - in shards, each a process flying every n-th aircraft with a
# few unpinned workers, so ctest -j runs them side by side on every core (FSIM_FLEET_SHARD, test_fleet.cpp).
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
add_executable(fsim_control_bench bench_control.cpp)
target_link_libraries(fsim_control_bench PRIVATE fsim::session fsim::control fsim::effects fsim::comm fsim::ipc fsim::sim fsim::io fsim::core)
target_compile_definitions(fsim_control_bench PRIVATE FSIM_TEST_JSBSIM_ROOT="${FSIM_JSBSIM_DATA_DIR}")
fsim_set_warnings(fsim_control_bench)
add_custom_command(TARGET fsim_control_bench POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:fsim_control_bench>)
add_test(NAME control_alloc COMMAND fsim_control_bench alloc)
set_tests_properties(control_alloc PROPERTIES LABELS conformance)

# The radar's cost and a digest of what it detected (tests/bench_radar.cpp), for A/B measurements of its code.
add_executable(fsim_radar_bench bench_radar.cpp)
target_link_libraries(fsim_radar_bench PRIVATE fsim::session fsim::ms fsim::control fsim::effects fsim::comm fsim::ipc fsim::sim fsim::io fsim::core)
target_compile_definitions(fsim_radar_bench PRIVATE FSIM_TEST_JSBSIM_ROOT="${FSIM_JSBSIM_DATA_DIR}")
fsim_set_warnings(fsim_radar_bench)
add_custom_command(TARGET fsim_radar_bench POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:fsim_radar_bench>)

# The public C++ SDK against fsim.dll (what a trainer links).
add_executable(fsim_sdk_tests test_sdk.cpp test_agents.cpp)
target_link_libraries(fsim_sdk_tests PRIVATE fsim::sdk Catch2::Catch2WithMain)
add_custom_command(TARGET fsim_sdk_tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:fsim> $<TARGET_FILE_DIR:fsim_sdk_tests>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:libJSBSim> $<TARGET_FILE_DIR:fsim_sdk_tests>)
target_compile_definitions(fsim_sdk_tests PRIVATE FSIM_TEST_JSBSIM_ROOT="${FSIM_JSBSIM_DATA_DIR}")
fsim_set_warnings(fsim_sdk_tests)
catch_discover_tests(fsim_sdk_tests ${FSIM_CATCH_DISCOVERY})

# The public C ABI, consumed from a plain C translation unit against fsim.dll.
add_executable(fsim_c_abi_test test_c_abi.c)
target_link_libraries(fsim_c_abi_test PRIVATE fsim::sdk)
set_target_properties(fsim_c_abi_test PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON)
add_test(NAME c_abi COMMAND fsim_c_abi_test "${FSIM_JSBSIM_DATA_DIR}")

# The crash handler writes a minidump (the crasher is expected to die).
add_executable(fsim_crash_test test_crash.cpp)
target_link_libraries(fsim_crash_test PRIVATE fsim::platform)
fsim_set_warnings(fsim_crash_test)
add_test(NAME crash_handler COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:fsim_crash_test> -DDIR=${CMAKE_BINARY_DIR}/crash-test -P ${CMAKE_SOURCE_DIR}/tests/crash_check.cmake)

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
             WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/examples/rust_trainer)
    # FSIM_BUILD_DIR gives build.rs bin/ (the DLLs) and lib/ (the import library).
    set_tests_properties(rust_example PROPERTIES ENVIRONMENT "FSIM_BUILD_DIR=${CMAKE_BINARY_DIR}" TIMEOUT 600)
endif()

# Viewer-side terrain decoding (VSG data types; no Vulkan device needed).
if(FSIM_WITH_RENDER)
    add_executable(fsim_render_tests test_terrain_render.cpp test_vehicle_visuals.cpp test_cocip.cpp test_contrail_observed.cpp)
    target_link_libraries(fsim_render_tests PRIVATE fsim::world fsim::core Catch2::Catch2WithMain)
    target_compile_definitions(fsim_render_tests PRIVATE FSIM_TEST_AIRCRAFT_DIR="${CMAKE_SOURCE_DIR}/aircraft")
    fsim_set_warnings(fsim_render_tests)
    fsim_relax_third_party_warnings(fsim_render_tests)
    catch_discover_tests(fsim_render_tests ${FSIM_CATCH_DISCOVERY})
    # The RCS engine (ADR-30, MS-3; tools/rcs) against the closed forms: on a GPU with ray queries, else skipped.
    if(TARGET fsim_rcs_engine)
        add_executable(fsim_rcs_tests ${CMAKE_SOURCE_DIR}/tools/rcs/test_rcs.cpp)
        target_link_libraries(fsim_rcs_tests PRIVATE fsim_rcs_engine fsim::ms fsim::core Catch2::Catch2WithMain)
        target_compile_definitions(fsim_rcs_tests PRIVATE FSIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
        fsim_set_warnings(fsim_rcs_tests)
        add_test(NAME rcs COMMAND fsim_rcs_tests)
        set_tests_properties(rcs PROPERTIES SKIP_RETURN_CODE 4)
    endif()
endif()

# Offscreen cameras through fsim_vision.dll (needs a Vulkan device).
if(FSIM_WITH_RENDER)
    add_executable(fsim_vision_test test_vision.cpp)
    target_link_libraries(fsim_vision_test PRIVATE fsim::sdk fsim::vision)
    add_custom_command(TARGET fsim_vision_test POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:fsim_vision> $<TARGET_FILE_DIR:fsim_vision_test>)
    fsim_set_warnings(fsim_vision_test)
    add_test(NAME vision COMMAND fsim_vision_test "${FSIM_JSBSIM_DATA_DIR}")
    set_tests_properties(vision PROPERTIES SKIP_RETURN_CODE 77) # no Vulkan device (CI runners)
    add_executable(fsim_vision_c_abi_test test_vision_c_abi.c)
    target_link_libraries(fsim_vision_c_abi_test PRIVATE fsim::sdk fsim::vision)
    set_target_properties(fsim_vision_c_abi_test PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON)
    add_test(NAME vision_c_abi COMMAND fsim_vision_c_abi_test "${FSIM_JSBSIM_DATA_DIR}")
    set_tests_properties(vision_c_abi PROPERTIES SKIP_RETURN_CODE 77)
endif()

# NVIDIA Flow's fires against the observed ones (tools/flow_calibrate check: its 12 cases), where Flow is built - it
# needs the GPU, some 20 s
if(TARGET flightsim-flow-calibrate)
    add_test(NAME "NVIDIA Flow: an engagement's fires against the observed - CCPS's fireballs, the photographed detonations, API RP 521's jet flames - each within its tolerance" COMMAND flightsim-flow-calibrate check WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()
