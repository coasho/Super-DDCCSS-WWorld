# Repository checks use only the Python standard library. Their registration
# is independent of whether a compatible CPython SDK module can be built.
set(FSIM_CHECK_PYTHON "" CACHE FILEPATH "Python 3.11+ interpreter for repository architecture checks")
set(_fsim_check_candidates "${FSIM_CHECK_PYTHON}" "${FSIM_PYTHON}" "${FSIM_PYTHON_EXECUTABLE}")
find_package(Python3 3.11 COMPONENTS Interpreter QUIET)
list(APPEND _fsim_check_candidates "${Python3_EXECUTABLE}")
set(_fsim_checks_python "")
foreach(_candidate IN LISTS _fsim_check_candidates)
    if(NOT _candidate)
        continue()
    endif()
    execute_process(COMMAND "${_candidate}" -c "import sys; sys.exit(0 if sys.version_info >= (3, 11) else 1)"
        RESULT_VARIABLE _probe OUTPUT_QUIET ERROR_QUIET)
    if(_probe EQUAL 0)
        set(_fsim_checks_python "${_candidate}")
        break()
    endif()
endforeach()
if(NOT _fsim_checks_python)
    message(WARNING "Architecture checks unavailable: set FSIM_CHECK_PYTHON to Python 3.11+ (standard library only). Native builds remain available.")
    return()
endif()
message(STATUS "Repository architecture checks: ${_fsim_checks_python} (independent of Python SDK)")
add_test(NAME weapon_quality_admission
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/weapon_quality/test_acceptance.py")
set_tests_properties(weapon_quality_admission PROPERTIES LABELS "sim-m03;testing" TIMEOUT 120)
foreach(_component_check IN ITEMS component_matrix component_report component_public full_chain component_reference_run
        component_references_kinematics component_references_effects)
    if(_component_check STREQUAL "component_references_kinematics" AND NOT FSIM_PYTHON_TEST_PATH)
        continue()
    endif()
    add_test(NAME "weapon_quality_${_component_check}"
        COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/weapon_quality/test_${_component_check}.py")
    set_tests_properties("weapon_quality_${_component_check}" PROPERTIES
        LABELS "sim-m03;testing;component-reference" TIMEOUT 180
        ENVIRONMENT "PYTHONPATH=${FSIM_PYTHON_TEST_PATH};TMP=D:/ENV/tmp;TEMP=D:/ENV/tmp;TORCH_EXTENSIONS_DIR=D:/ENV/tmp")
endforeach()
foreach(_reference_kind IN ITEMS effects kinematics)
    if(_reference_kind STREQUAL "effects")
        set(_reference_target fsim_component_effect_reference_native_test)
        set(_reference_test component_effect_reference_native)
    else()
        set(_reference_target fsim_component_reference_native_test)
        set(_reference_test component_reference_native)
    endif()
    if(TARGET ${_reference_target})
        add_test(NAME "${_reference_test}"
            COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/weapon_quality/component_reference_run.py"
                --kind "${_reference_kind}" --executable "$<TARGET_FILE:${_reference_target}>"
                --output "${CMAKE_BINARY_DIR}/validation/component_reference_runs/${_reference_kind}")
        set_tests_properties("${_reference_test}" PROPERTIES
            LABELS "payload-components;sim-m03;component-reference" TIMEOUT 360 RUN_SERIAL TRUE
            ENVIRONMENT "PYTHONPATH=${FSIM_PYTHON_TEST_PATH};TMP=D:/ENV/tmp;TEMP=D:/ENV/tmp;TORCH_EXTENSIONS_DIR=D:/ENV/tmp")
    endif()
endforeach()
add_test(NAME weapon_quality_delivery
    COMMAND "${_fsim_checks_python}" -B -W error::ResourceWarning
        "${CMAKE_SOURCE_DIR}/tools/validation/weapon_quality/test_delivery.py")
set_tests_properties(weapon_quality_delivery PROPERTIES LABELS "sim-m03;package;testing" TIMEOUT 120)
add_test(NAME weapon_quality_maps
    COMMAND "${_fsim_checks_python}" -B -W error::ResourceWarning
        "${CMAKE_SOURCE_DIR}/tools/validation/weapon_quality/test_maps.py")
set_tests_properties(weapon_quality_maps PROPERTIES LABELS "sim-m03;package;testing" TIMEOUT 120)
add_test(NAME weapon_quality_explosive_fixtures
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/weapon_quality/test_explosives.py")
set_tests_properties(weapon_quality_explosive_fixtures PROPERTIES LABELS "sim-m03;testing" TIMEOUT 120)
add_test(NAME weapon_quality_effects_population
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/weapon_quality/test_effects_population.py")
set_tests_properties(weapon_quality_effects_population PROPERTIES LABELS "sim-m03;testing" TIMEOUT 120)
add_test(NAME weapon_quality_liquid_population
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/weapon_quality/test_liquid_population.py")
set_tests_properties(weapon_quality_liquid_population PROPERTIES LABELS "sim-m03;testing" TIMEOUT 120)
add_test(NAME weapon_quality_external_tanks
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/weapon_quality/test_external_tanks.py")
set_tests_properties(weapon_quality_external_tanks PROPERTIES LABELS "sim-m03;testing" TIMEOUT 120)
add_test(NAME weapon_quality_liquid_vessel_policy
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/weapon_quality/test_liquid_vessel_policy.py")
set_tests_properties(weapon_quality_liquid_vessel_policy PROPERTIES LABELS "sim-m03;testing" TIMEOUT 120)
# The complete model campaign supplies this existing authoring dependency path.
# Keep the ordinary standard-library repository checks independently available.
if(FSIM_PYTHON_TEST_PATH)
    add_test(NAME weapon_quality_model_methods
        COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/weapon_quality/model_methods.py"
            --source-root "${CMAKE_SOURCE_DIR}" --build-dir "${CMAKE_BINARY_DIR}"
            --dependency-path "${FSIM_PYTHON_TEST_PATH}")
    set_tests_properties(weapon_quality_model_methods PROPERTIES LABELS "sim-m03;hangar;testing" TIMEOUT 180 RUN_SERIAL TRUE)
endif()
add_test(NAME weapon_quality_backend_epoch
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/weapon_quality/test_backend_epoch.py")
set_tests_properties(weapon_quality_backend_epoch PROPERTIES LABELS "sim-m03;testing" TIMEOUT 120)
  add_test(NAME weapon_quality_countermeasures
      COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/weapon_quality/test_countermeasures.py")
  set_tests_properties(weapon_quality_countermeasures PROPERTIES LABELS "sim-m03;testing" TIMEOUT 120)
  add_test(NAME weapon_quality_authored_fits
      COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/weapon_quality/test_fits.py")
  set_tests_properties(weapon_quality_authored_fits PROPERTIES LABELS "sim-m03;testing" TIMEOUT 120)
if(FSIM_WITH_RENDER AND TARGET fsim_python)
    add_test(NAME aam_campaign_tools
        COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/viewer_perf/test_aam_campaign.py")
    set_tests_properties(aam_campaign_tools PROPERTIES LABELS "sim-m03;render;testing" TIMEOUT 120
        ENVIRONMENT "PYTHONPATH=${CMAKE_BINARY_DIR}/python;TMP=D:/ENV/tmp;TEMP=D:/ENV/tmp;TORCH_EXTENSIONS_DIR=D:/ENV/tmp")
endif()
if(TARGET flightsim-showcase)
    add_test(NAME engagement_interactive
        COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/launch/test_engagement.py"
            --build "${CMAKE_BINARY_DIR}" --scene "${CMAKE_SOURCE_DIR}/tools/validation/viewer_perf/engagement.json"
            --out "${CMAKE_BINARY_DIR}/validation/engagement-interactive")
    set_tests_properties(engagement_interactive PROPERTIES
        LABELS "sim-m03;native-engagement;launcher" TIMEOUT 90 RUN_SERIAL TRUE
        ENVIRONMENT "TMP=D:/ENV/tmp;TEMP=D:/ENV/tmp;TORCH_EXTENSIONS_DIR=D:/ENV/tmp")
endif()
add_test(NAME architecture_boundary
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/architecture/check.py" --repo "${CMAKE_SOURCE_DIR}")
add_test(NAME architecture_checker
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/architecture/test_check.py")
add_test(NAME sdk_guard_checker
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/boundary/test_boundary.py")
set_tests_properties(architecture_boundary architecture_checker sdk_guard_checker
    PROPERTIES LABELS architecture TIMEOUT 120)
# The configured build graph (tools/architecture/README.md, "Reconciling with the configured build"; ARCH-10): this
# configure's own CMake File API code model and cache, held to the source graph the gate above reads from every branch.
# cmake_file_api() (CMake 3.27+) requests it for this run; without that command the query file is answered from the
# next configure, and the gate fails until a reply exists - a preset without one is pending, not passed.
# The gate needs CMake 4.2 or later (HC-43, the user's compatibility decision): native builds stay supported from CMake
# 3.25, and with an older CMake the test is still registered and fails with an actionable diagnostic, leaving this
# preset's configured-graph acceptance pending rather than passed.
if(CMAKE_VERSION VERSION_LESS 4.2)
    message(WARNING "architecture_configured needs CMake 4.2 or later; with CMake ${CMAKE_VERSION} it fails, and this "
                    "preset's configured-graph acceptance stays pending (native builds remain supported from CMake 3.25)")
endif()
if(COMMAND cmake_file_api)
    cmake_file_api(QUERY API_VERSION 1 CODEMODEL 2 CACHE 2)
else()
    file(WRITE "${CMAKE_BINARY_DIR}/.cmake/api/v1/query/codemodel-v2" "")
    file(WRITE "${CMAKE_BINARY_DIR}/.cmake/api/v1/query/cache-v2" "")
endif()
add_test(NAME architecture_configured
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/architecture/configured.py" --repo "${CMAKE_SOURCE_DIR}"
            --build "${CMAKE_BINARY_DIR}")
add_test(NAME architecture_configured_checker
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/architecture/test_configured.py")
set_tests_properties(architecture_configured architecture_configured_checker PROPERTIES LABELS architecture TIMEOUT 120)
# The mission-training contract (docs/sdk/mission-training.md): its schema's dialect, its fixtures and its pages, and
# the checker's own refusals. They hold the contract's documents to the contract; no SDK binary is loaded.
add_test(NAME ma_contract
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/ma_contract/check.py" --repo "${CMAKE_SOURCE_DIR}")
add_test(NAME ma_contract_checker
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/ma_contract/test_check.py")
set_tests_properties(ma_contract ma_contract_checker PROPERTIES LABELS ma-training TIMEOUT 120)
# Change-scoped test selection (tools/validation/testing; docs/development.md, "Change-scoped testing"): the selector's rules
# against synthetic inventories, and its rules file against the repository and this build's own CTest inventory.
add_test(NAME testing_scope_checker
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/testing/test_scope.py" --test-dir "${CMAKE_BINARY_DIR}"
            --ctest "${CMAKE_CTEST_COMMAND}")
set_tests_properties(testing_scope_checker PROPERTIES LABELS testing TIMEOUT 120)
add_test(NAME project_bootstrap_checker
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/tests/test_project_bootstrap.py")
set_tests_properties(project_bootstrap_checker PROPERTIES LABELS bootstrap TIMEOUT 120)
# Offline propulsion-reference guards use only the standard library.
add_test(NAME missile_flight_reference_checker
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/missile/test_reference.py")
set_tests_properties(missile_flight_reference_checker
    PROPERTIES LABELS "simulation;model-reference" TIMEOUT 30)
add_test(NAME resource_rcs_binding
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/resources/check_rcs.py"
            --aircraft-root "${CMAKE_SOURCE_DIR}/aircraft" --output "${CMAKE_BINARY_DIR}/rcs-asset-binding.json")
add_test(NAME resource_rcs_binding_checker
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/resources/test_rcs_binding.py")
set_tests_properties(resource_rcs_binding PROPERTIES LABELS "simulation;resources;model-reference" TIMEOUT 300)
set_tests_properties(resource_rcs_binding_checker PROPERTIES LABELS "simulation;resources;model-reference" TIMEOUT 30)
# Separate versioned extension contracts; these checks do not execute SDK handlers.
add_test(NAME ma_engagement_contract
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/ma_engagement/check.py" --repo "${CMAKE_SOURCE_DIR}")
add_test(NAME ma_engagement_checker
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/ma_engagement/test_check.py")
set_tests_properties(ma_engagement_contract ma_engagement_checker PROPERTIES LABELS ma-training TIMEOUT 120)
