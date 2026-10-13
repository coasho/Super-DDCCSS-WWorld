# The MA training interfaces from installed packages alone: install the SDK and the Python package into private
# prefixes, then build and run separate C++, C99 and Python consumers of the MA-service binding and the legal reports
# (docs/sdk/mission-training.md, sections 9 and 10). This gate never changes the source tree, uses no development DLL
# on PATH and no source header, and records what it ran against.
cmake_minimum_required(VERSION 3.25)
foreach(_required SOURCE_DIR BUILD_DIR TOOLCHAIN_FILE GENERATOR UCRT64_ROOT)
    if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
        message(FATAL_ERROR "RunMissionPackageConsumer: ${_required} is required")
    endif()
endforeach()
file(REAL_PATH "${BUILD_DIR}" _build)
set(_work "${_build}/mission-package-consumer")
# The sole removable directory is a child owned by this gate, never a caller's prefix.
cmake_path(IS_PREFIX _build "${_work}" NORMALIZE _inside_build)
if(NOT _inside_build OR _work STREQUAL _build OR NOT IS_DIRECTORY "${_build}")
    message(FATAL_ERROR "RunMissionPackageConsumer: invalid private work directory: ${_work}")
endif()
file(REMOVE_RECURSE "${_work}")
set(_prefix "${_work}/installed SDK")
set(_python_prefix "${_work}/installed python")
set(_consumer_build "${_work}/consumer build")
set(_data "${_prefix}/share/flightsim")
file(MAKE_DIRECTORY "${_work}")

# The interpreter the Python SDK was built for, as its configuration recorded it. All three languages are this gate's:
# a tree without the Python package fails here, it does not pass on two.
set(_python_record "${_build}/python/fsim-python.json")
if(NOT EXISTS "${_python_record}")
    message(FATAL_ERROR "RunMissionPackageConsumer: the Python SDK is not built in ${_build} (${_python_record} is missing)")
endif()
file(READ "${_python_record}" _python_json)
string(JSON _python GET "${_python_json}" executable)
if(NOT EXISTS "${_python}")
    message(FATAL_ERROR "RunMissionPackageConsumer: the recorded Python interpreter does not exist: ${_python}")
endif()

function(run_checked label)
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE _result OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    file(WRITE "${_work}/${label}.log" "${_out}${_err}")
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "${label} failed (${_result}); see ${_work}/${label}.log\n${_out}${_err}")
    endif()
endfunction()
foreach(_component sdk python)
    if(_component STREQUAL "sdk")
        set(_install "${CMAKE_COMMAND}" --install "${BUILD_DIR}" --component sdk --prefix "${_prefix}")
    else()
        set(_install "${CMAKE_COMMAND}" --install "${BUILD_DIR}" --component python --prefix "${_python_prefix}")
    endif()
    if(CONFIG)
        list(APPEND _install --config "${CONFIG}")
    endif()
    run_checked(install-${_component} ${_install})
endforeach()

# Provenance: what was installed is this tree's interface and this build's library, byte for byte.
set(_manifest "")
function(check_file source installed root)
    if(NOT EXISTS "${installed}" OR IS_DIRECTORY "${installed}")
        message(FATAL_ERROR "installed file missing: ${installed}")
    endif()
    file(SHA256 "${source}" _source_sha)
    file(SHA256 "${installed}" _installed_sha)
    if(NOT _source_sha STREQUAL _installed_sha)
        message(FATAL_ERROR "installed file differs from ${source}: ${installed}")
    endif()
    file(RELATIVE_PATH _relative "${root}" "${installed}")
    set(_manifest "${_manifest}${_installed_sha}  ${_relative}\n" PARENT_SCOPE)
endfunction()
file(GLOB _headers LIST_DIRECTORIES false RELATIVE "${SOURCE_DIR}/include/fsim" "${SOURCE_DIR}/include/fsim/*.h")
foreach(_required_header Agent.h MAReports.h MAService.h fsim_c.h)
    if(NOT _required_header IN_LIST _headers)
        message(FATAL_ERROR "public header missing from the source tree: ${_required_header}")
    endif()
endforeach()
foreach(_header IN LISTS _headers)
    check_file("${SOURCE_DIR}/include/fsim/${_header}" "${_prefix}/include/fsim/${_header}" "${_work}")
endforeach()
check_file("${_build}/bin/libfsim.dll" "${_prefix}/bin/libfsim.dll" "${_work}")
foreach(_module __init__.py agent.py world.py isolation.py)
    check_file("${SOURCE_DIR}/python/fsim/${_module}" "${_python_prefix}/fsim/${_module}" "${_work}")
endforeach()
check_file("${_build}/python/fsim/_native.pyd" "${_python_prefix}/fsim/_native.pyd" "${_work}")
check_file("${_build}/python/fsim/bin/libfsim.dll" "${_python_prefix}/fsim/bin/libfsim.dll" "${_work}")
file(WRITE "${_work}/provenance.sha256" "${_manifest}")

set(_configure "${CMAKE_COMMAND}" -S "${SOURCE_DIR}/tests/mission_package_consumer" -B "${_consumer_build}"
    -G "${GENERATOR}" "-DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_FILE}" "-DFSIM_UCRT64_ROOT=${UCRT64_ROOT}"
    "-Dfsim_DIR=${_prefix}/lib/cmake/fsim" "-DCMAKE_PREFIX_PATH=${_prefix}"
    -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF -DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF
    "-DCMAKE_BUILD_TYPE=${CONFIG}" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON)
if(MAKE_PROGRAM)
    list(APPEND _configure "-DCMAKE_MAKE_PROGRAM=${MAKE_PROGRAM}")
endif()
run_checked(configure ${_configure})
set(_compile "${CMAKE_COMMAND}" --build "${_consumer_build}")
if(CONFIG)
    list(APPEND _compile --config "${CONFIG}")
endif()
run_checked(build ${_compile})
# No consumer was compiled against a source header: the only fsim include directory in its build is the installed one.
file(READ "${_consumer_build}/compile_commands.json" _commands)
string(FIND "${_commands}" "${SOURCE_DIR}/include" _source_include)
string(REGEX MATCH "mission-package-consumer/installed[^/]*SDK/include" _installed_include "${_commands}")
if(NOT _source_include EQUAL -1 OR NOT _installed_include)
    message(FATAL_ERROR "the consumers were not compiled against the installed headers alone; see ${_consumer_build}/compile_commands.json")
endif()

set(_env "FSIM_AIRCRAFT_PATH=${_data}/aircraft;${_data}/aircraft/stores")
foreach(_language cpp c)
    # Only installed bin and Windows system directories participate in DLL lookup.
    execute_process(COMMAND "${CMAKE_COMMAND}" -E env
        "${_env}" "PATH=${_prefix}/bin;$ENV{SystemRoot}/System32;$ENV{SystemRoot}"
        "${_prefix}/bin/ma_consumer_${_language}.exe" "${_prefix}"
        WORKING_DIRECTORY "${_work}" RESULT_VARIABLE _result OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    file(WRITE "${_work}/run-${_language}.log" "${_out}${_err}")
    if(NOT _result STREQUAL "0")
        message(FATAL_ERROR "installed MA consumer (${_language}): expected 0, got ${_result}\n${_out}${_err}")
    endif()
    message(STATUS "installed MA consumer (${_language}): exit 0\n${_out}")
endforeach()
# The installed Python package alone: its own library and data, no development tree on any path. The script is copied
# out of the source tree first, so its directory (the interpreter's first path entry) is this gate's own.
file(COPY_FILE "${SOURCE_DIR}/tests/mission_package_consumer/ma_python.py" "${_work}/ma_python.py")
execute_process(COMMAND "${CMAKE_COMMAND}" -E env --unset=FSIM_DLL_DIR --unset=FSIM_AIRCRAFT_PATH --unset=PYTHONHOME
        "PYTHONPATH=${_python_prefix}" "PATH=$ENV{SystemRoot}/System32;$ENV{SystemRoot}"
        "${_python}" -s -B "${_work}/ma_python.py" "${_python_prefix}"
    WORKING_DIRECTORY "${_work}" RESULT_VARIABLE _result OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
file(WRITE "${_work}/run-python.log" "${_out}${_err}")
if(NOT _result STREQUAL "0")
    message(FATAL_ERROR "installed MA consumer (python): expected 0, got ${_result}\n${_out}${_err}")
endif()
message(STATUS "installed MA consumer (python): exit 0\n${_out}")
message(STATUS "installed MA consumers passed in C++, C99 and Python; provenance: ${_work}/provenance.sha256")
