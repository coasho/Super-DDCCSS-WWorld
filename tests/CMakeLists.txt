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

add_executable(fsim_strike_capture_test test_strike_capture.cpp)
target_link_libraries(fsim_strike_capture_test PRIVATE fsim::ipc fsim::core)
fsim_set_warnings(fsim_strike_capture_test)
add_test(NAME strike_capture COMMAND fsim_strike_capture_test)
set_tests_properties(strike_capture PROPERTIES LABELS "showcase;native-engagement")

if(TARGET flightsim-showcase)
    add_executable(fsim_showcase_native_test test_showcase_native.cpp)
    target_link_libraries(fsim_showcase_native_test PRIVATE fsim::ipc fsim::core fsim::sdk)
    fsim_set_warnings(fsim_showcase_native_test)
    add_test(NAME showcase_native COMMAND fsim_showcase_native_test
        $<TARGET_FILE:flightsim-showcase> ${CMAKE_BINARY_DIR}/showcase-native-test)
    set_tests_properties(showcase_native PROPERTIES LABELS "showcase;native-engagement"
        TIMEOUT 120 WORKING_DIRECTORY $<TARGET_FILE_DIR:flightsim-showcase>)
endif()

add_executable(fsim_tests
    test_units.cpp
    test_damage.cpp
    test_impact.cpp
    test_mass_change.cpp
    test_store_binding.cpp
    test_store_drag.cpp
    test_rng.cpp
    test_registry.cpp
    test_vehicle_pool.cpp
    test_vecenv.cpp
    test_axes.cpp
    test_capabilities.cpp
    test_conformance.cpp
    test_ma_lifecycle.cpp
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

# Explicit MA-service binding through the public C++ SDK (docs/sdk/mission-training.md, section 9): configured
# ownership whatever order the agents were made in, refused overlaps and foreign controllers, the legacy path as it was.
add_executable(fsim_ma_service_binding_test test_ma_service_binding.cpp)
target_link_libraries(fsim_ma_service_binding_test PRIVATE fsim::sdk Catch2::Catch2WithMain)
fsim_set_warnings(fsim_ma_service_binding_test)
add_test(NAME ma_service_binding COMMAND fsim_ma_service_binding_test)
set_tests_properties(ma_service_binding PROPERTIES LABELS ma-training TIMEOUT 300)

# Legal reports with their provenance through the public C++ SDK (docs/sdk/mission-training.md, section 10): unknown,
# stale, unavailable and undelivered stay what they are, and hidden truth changes nothing an agent reads.
add_executable(fsim_ma_reports_test test_ma_reports.cpp)
target_link_libraries(fsim_ma_reports_test PRIVATE fsim::sdk Catch2::Catch2WithMain)
fsim_set_warnings(fsim_ma_reports_test)
add_test(NAME ma_reports COMMAND fsim_ma_reports_test)
set_tests_properties(ma_reports PROPERTIES LABELS ma-training TIMEOUT 300)

# The same contracts through the C ABI, from a plain C99 translation unit.
add_executable(fsim_ma_c_abi_test test_ma_c_abi.c)
target_link_libraries(fsim_ma_c_abi_test PRIVATE fsim::sdk)
set_target_properties(fsim_ma_c_abi_test PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON)
add_test(NAME ma_c_abi COMMAND fsim_ma_c_abi_test "${FSIM_JSBSIM_DATA_DIR}")
set_tests_properties(ma_c_abi PROPERTIES LABELS ma-training TIMEOUT 300)

# Same-build existing-native-state input for M4; this target does not implement episode ownership/reset.
add_library(fsim_native_state_reset_fixture STATIC
    fixtures/mission_training/NativeStateReset.cpp
    fixtures/mission_training/NativeStateResetCapture.cpp)
target_link_libraries(fsim_native_state_reset_fixture PUBLIC fsim::sdk fsim::session)
fsim_set_warnings(fsim_native_state_reset_fixture)
add_executable(fsim_native_state_reset_fixture_test test_native_state_reset_fixture.cpp)
target_link_libraries(fsim_native_state_reset_fixture_test PRIVATE fsim_native_state_reset_fixture)
target_link_libraries(fsim_native_state_reset_fixture_test PRIVATE fsim::sdk fsim::session Catch2::Catch2WithMain)
fsim_set_warnings(fsim_native_state_reset_fixture_test)
add_test(NAME native_state_reset_fixture COMMAND fsim_native_state_reset_fixture_test)
set_tests_properties(native_state_reset_fixture PROPERTIES LABELS "ma-training;native-engagement" TIMEOUT 300 RUN_SERIAL TRUE)

# Independent mission episodes (M4; docs/sdk/mission-training.md, sections 4 and 8): one World per episode, rebuilt
# whole on reset, generation-bound children, seed streams - and the supplied native-state fixture applied to the
# episode's own reset.
add_executable(fsim_mission_episode_test test_mission_episode.cpp)
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

# Simulation construction, fixed state/transport ABI and output-only diagnostics stay independent of M5.
add_executable(fsim_simulation_interface_boundary_test
    test_episode_world_options.cpp
    test_state_layout.cpp
    test_simulation_output_boundary.cpp)
target_link_libraries(fsim_simulation_interface_boundary_test PRIVATE fsim::env fsim_native_state_reset_fixture
    fsim::sdk fsim::session fsim::ipc Catch2::Catch2WithMain)
target_compile_definitions(fsim_simulation_interface_boundary_test PRIVATE
    FSIM_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/simulation-interface-boundary-test")
fsim_set_warnings(fsim_simulation_interface_boundary_test)
add_test(NAME simulation_interface_boundary COMMAND fsim_simulation_interface_boundary_test)
set_tests_properties(simulation_interface_boundary PROPERTIES LABELS "architecture;simulation" TIMEOUT 300 RUN_SERIAL TRUE)

# Own-platform equipment access and report provenance through public C++ and C99 interfaces.
add_executable(fsim_ma_equipment_test test_ma_equipment.cpp)
target_link_libraries(fsim_ma_equipment_test PRIVATE fsim::sdk Catch2::Catch2WithMain)
fsim_set_warnings(fsim_ma_equipment_test)
add_test(NAME ma_equipment COMMAND fsim_ma_equipment_test)
set_tests_properties(ma_equipment PROPERTIES LABELS "ma-training;native-engagement" TIMEOUT 300)

add_executable(fsim_ma_equipment_c_test test_ma_equipment_c.c)
target_link_libraries(fsim_ma_equipment_c_test PRIVATE fsim::sdk)
set_target_properties(fsim_ma_equipment_c_test PROPERTIES C_STANDARD 99 C_STANDARD_REQUIRED ON)
add_test(NAME ma_equipment_c COMMAND fsim_ma_equipment_c_test "${FSIM_JSBSIM_DATA_DIR}")
set_tests_properties(ma_equipment_c PROPERTIES LABELS "ma-training;native-engagement" TIMEOUT 300)

# Public C control/Mission adapter behavior across the shared SDK boundary.
add_executable(fsim_adapter_compatibility_test test_adapter_compatibility.cpp)
target_link_libraries(fsim_adapter_compatibility_test PRIVATE fsim::sdk)
fsim_set_warnings(fsim_adapter_compatibility_test)
add_test(NAME adapter_compatibility COMMAND fsim_adapter_compatibility_test)

# Public World, Scenario and Environment adapter behavior across the SDK boundary.
add_executable(fsim_world_adapter_compatibility_test test_world_adapter_compatibility.cpp)
target_link_libraries(fsim_world_adapter_compatibility_test PRIVATE fsim::sdk)
fsim_set_warnings(fsim_world_adapter_compatibility_test)
add_test(NAME world_adapter_compatibility COMMAND fsim_world_adapter_compatibility_test)

# Public VecEnv and Recording adapter lifetimes and error behavior.
add_executable(fsim_batch_recording_adapter_compatibility_test test_batch_recording_adapter_compatibility.cpp)
target_link_libraries(fsim_batch_recording_adapter_compatibility_test PRIVATE fsim::sdk)
fsim_set_warnings(fsim_batch_recording_adapter_compatibility_test)
add_test(NAME batch_recording_adapter_compatibility COMMAND fsim_batch_recording_adapter_compatibility_test "${FSIM_JSBSIM_DATA_DIR}")

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
    # Fullscreen screenshot readback must capture the frame just presented (needs a Vulkan window).
    add_executable(fsim_viewer_screenshot_test test_viewer_screenshot.cpp)
    target_link_libraries(fsim_viewer_screenshot_test PRIVATE fsim::render vsgXchange::vsgXchange)
    fsim_set_warnings(fsim_viewer_screenshot_test)
    fsim_relax_third_party_warnings(fsim_viewer_screenshot_test)
    add_test(NAME viewer_screenshot COMMAND fsim_viewer_screenshot_test "${CMAKE_BINARY_DIR}/viewer-screenshot-test")
    set_tests_properties(viewer_screenshot PROPERTIES SKIP_RETURN_CODE 77 RUN_SERIAL TRUE TIMEOUT 60)
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
