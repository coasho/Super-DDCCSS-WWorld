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
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/boundary/test_boundary.py")
set_tests_properties(architecture_boundary architecture_checker sdk_guard_checker
    PROPERTIES LABELS architecture TIMEOUT 120)
# The mission-training contract (docs/sdk/mission-training.md): its schema's dialect, its fixtures and its pages, and
# the checker's own refusals. They hold the contract's documents to the contract; no SDK binary is loaded.
add_test(NAME ma_contract
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/ma_contract/check.py" --repo "${CMAKE_SOURCE_DIR}")
add_test(NAME ma_contract_checker
    COMMAND "${_fsim_checks_python}" -B "${CMAKE_SOURCE_DIR}/tools/ma_contract/test_check.py")
set_tests_properties(ma_contract ma_contract_checker PROPERTIES LABELS ma-training TIMEOUT 120)
