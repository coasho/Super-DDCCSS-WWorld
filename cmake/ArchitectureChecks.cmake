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
# Separate versioned extension contracts; these checks do not execute SDK handlers.
add_test(NAME ma_engagement_contract
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/ma_engagement/check.py" --repo "${CMAKE_SOURCE_DIR}")
add_test(NAME ma_engagement_checker
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/validation/ma_engagement/test_check.py")
set_tests_properties(ma_engagement_contract ma_engagement_checker PROPERTIES LABELS ma-training TIMEOUT 120)
