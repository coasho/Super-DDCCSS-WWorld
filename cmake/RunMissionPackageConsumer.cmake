# The MA training interfaces from installed packages alone: install the SDK and the Python package into private
# prefixes, then build and run separate C++, C99 and Python consumers of the MA-service binding, the legal reports and
# the mission episode and batch family (docs/sdk/mission-training.md, sections 3 to 18). This gate never changes the source tree, uses no development DLL
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
foreach(_required_header Agent.h MAReports.h MAService.h MissionBatch.h MissionEpisode.h fsim_c.h fsim_mission_env.h)
    if(NOT _required_header IN_LIST _headers)
        message(FATAL_ERROR "public header missing from the source tree: ${_required_header}")
    endif()
endforeach()
foreach(_header IN LISTS _headers)
    check_file("${SOURCE_DIR}/include/fsim/${_header}" "${_prefix}/include/fsim/${_header}" "${_work}")
endforeach()
check_file("${_build}/bin/libfsim.dll" "${_prefix}/bin/libfsim.dll" "${_work}")
foreach(_module __init__.py agent.py world.py isolation.py mission.py training.py)
    check_file("${SOURCE_DIR}/python/fsim/${_module}" "${_python_prefix}/fsim/${_module}" "${_work}")
endforeach()
check_file("${_build}/python/fsim/_native.pyd" "${_python_prefix}/fsim/_native.pyd" "${_work}")
check_file("${_build}/python/fsim/bin/libfsim.dll" "${_python_prefix}/fsim/bin/libfsim.dll" "${_work}")
# The runtime data the joint chains fly - the F-16C and the released stores' flight models - as the staging rules lay
# it out: cmake/StageDesigns.cmake, applied here to the source tree's copies of exactly those designs. That expected
# inventory - not whatever happens to be installed - is the list: each of its files must be installed in both
# packages, byte for byte, and an omitted file fails here instead of dropping out of the list. The Python consumer
# checks the same inventory again (runtime-data.sha256) before it makes any World, and so do its refusal probes below.
set(_expected_source "${_work}/expected source")
set(_expected "${_work}/expected aircraft")
file(MAKE_DIRECTORY "${_expected_source}")
file(COPY "${SOURCE_DIR}/aircraft/f16c" "${SOURCE_DIR}/aircraft/stores" DESTINATION "${_expected_source}")
run_checked(stage-expected "${CMAKE_COMMAND}" "-DSRC=${_expected_source}" "-DDEST=${_expected}" -P "${SOURCE_DIR}/cmake/StageDesigns.cmake")
file(GLOB_RECURSE _runtime_files LIST_DIRECTORIES false RELATIVE "${_expected}" "${_expected}/*")
foreach(_required f16c/f16c.xml f16c/Engines/f16c_engine0.xml)
    if(NOT _required IN_LIST _runtime_files)
        message(FATAL_ERROR "the staging rules no longer stage ${_required}: the expected runtime inventory is wrong")
    endif()
endforeach()
set(_runtime_inventory "")
foreach(_file IN LISTS _runtime_files)
    file(SHA256 "${_expected}/${_file}" _sha)
    string(APPEND _runtime_inventory "${_sha}  ${_file}\n")
    foreach(_aircraft_root "${_prefix}/share/flightsim/aircraft" "${_python_prefix}/fsim/share/flightsim/aircraft")
        check_file("${_expected}/${_file}" "${_aircraft_root}/${_file}" "${_work}")
    endforeach()
endforeach()
list(LENGTH _runtime_files _runtime_count)
file(WRITE "${_work}/runtime-data.sha256" "${_runtime_inventory}")
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
# The joint chains (M8; tests/fixtures/mission_training/joint, tools/ma_contract/joint_chain.py), copied out of the source
# tree: every consumer replays each one and writes its trace, compared below byte for byte.
set(_chains joint lossy)
file(REMOVE_RECURSE "${_work}/joint")
file(COPY "${SOURCE_DIR}/tests/fixtures/mission_training/joint" DESTINATION "${_work}")
foreach(_language cpp c)
    set(_chain_args)
    foreach(_chain IN LISTS _chains)
        file(REMOVE "${_work}/trace-${_chain}-${_language}.trace")
        list(APPEND _chain_args "${_work}/joint/chain.${_chain}.txt" "${_work}/trace-${_chain}-${_language}.trace")
    endforeach()
    # Only installed bin and Windows system directories participate in DLL lookup.
    execute_process(COMMAND "${CMAKE_COMMAND}" -E env
        "${_env}" "PATH=${_prefix}/bin;$ENV{SystemRoot}/System32;$ENV{SystemRoot}"
        "${_prefix}/bin/ma_consumer_${_language}.exe" "${_prefix}" ${_chain_args}
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
# Its aircraft and stores are the installed package's: a development library's source aircraft directory would
# otherwise come first (src/io/AssetResolver.cpp). The isolated actor's process inherits the same environment.
set(_python_aircraft "${_python_prefix}/fsim/share/flightsim/aircraft")
set(_python_env "FSIM_AIRCRAFT_PATH=${_python_aircraft};${_python_aircraft}/stores")
set(_chain_args)
foreach(_chain IN LISTS _chains)
    file(REMOVE "${_work}/trace-${_chain}-python.trace" "${_work}/trace-${_chain}-python-isolated.trace")
    list(APPEND _chain_args "${_work}/joint/chain.${_chain}.txt" "${_work}/trace-${_chain}-python.trace"
                            "${_work}/trace-${_chain}-python-isolated.trace")
endforeach()
execute_process(COMMAND "${CMAKE_COMMAND}" -E env --unset=FSIM_DLL_DIR --unset=PYTHONHOME "${_python_env}"
        "PYTHONPATH=${_python_prefix}" "PATH=$ENV{SystemRoot}/System32;$ENV{SystemRoot}"
        "${_python}" -s -B "${_work}/ma_python.py" "${_python_prefix}" "${_work}/runtime-data.sha256" ${_chain_args}
    WORKING_DIRECTORY "${_work}" RESULT_VARIABLE _result OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
file(WRITE "${_work}/run-python.log" "${_out}${_err}")
if(NOT _result STREQUAL "0")
    message(FATAL_ERROR "installed MA consumer (python): expected 0, got ${_result}\n${_out}${_err}")
endif()
message(STATUS "installed MA consumer (python): exit 0\n${_out}")
# J10 (generic): the joint chains through installed C++, C99 and Python - in-process and with an isolated actor - wrote
# the same trace, byte for byte.
foreach(_chain IN LISTS _chains)
    set(_digests)
    foreach(_who cpp c python python-isolated)
        set(_trace "${_work}/trace-${_chain}-${_who}.trace")
        if(NOT EXISTS "${_trace}")
            message(FATAL_ERROR "installed joint chain ${_chain}: the ${_who} consumer wrote no trace")
        endif()
        file(SHA256 "${_trace}" _digest)
        list(APPEND _digests "${_digest}")
    endforeach()
    list(REMOVE_DUPLICATES _digests)
    list(LENGTH _digests _distinct)
    if(NOT _distinct EQUAL 1)
        message(FATAL_ERROR "installed joint chain ${_chain}: the consumers' traces differ (${_digests}); see ${_work}/trace-${_chain}-*.trace")
    endif()
    message(STATUS "installed joint chain ${_chain}: C++, C99, Python and an isolated Python actor wrote one trace (${_digests})")
endforeach()
# The installed data is the data flown. Each probe changes one installed file of the Python package, runs the consumer
# on the joint chain and restores the file:
#   - the F-16C definition missing, and an engine file it names missing: the preflight (the same function the normal
#     run calls first) refuses before any World is made - exit 4, naming the installed path - although a development
#     library would find the source tree's copies after the installed ones (src/io/AssetResolver.cpp, kept as it is);
#   - the definition corrupted: the preflight refuses it (exit 4) for its hash; and with the presence check alone
#     (--probe-native-read) the corrupted file reaches the SDK, which refuses the episode (exit 3): the installed copy
#     is the one read.
# Afterwards the whole inventory is held to the expected one again.
set(_probes "missing|f16c/f16c.xml|--probe|4" "missing|f16c/Engines/f16c_engine0.xml|--probe|4"
            "corrupted|f16c/f16c.xml|--probe|4" "corrupted|f16c/f16c.xml|--probe-native-read|3")
set(_probe_results)
foreach(_probe IN LISTS _probes)
    string(REPLACE "|" ";" _probe "${_probe}")
    list(GET _probe 0 _change)
    list(GET _probe 1 _relative)
    list(GET _probe 2 _mode)
    list(GET _probe 3 _expected_exit)
    set(_target "${_python_aircraft}/${_relative}")
    string(MAKE_C_IDENTIFIER "${_change}-${_relative}${_mode}" _probe_name)
    file(COPY_FILE "${_target}" "${_work}/probe.installed")
    if(_change STREQUAL "corrupted")
        file(WRITE "${_target}" "<not-an-aircraft/>\n")
    else()
        file(REMOVE "${_target}")
    endif()
    execute_process(COMMAND "${CMAKE_COMMAND}" -E env --unset=FSIM_DLL_DIR --unset=PYTHONHOME "${_python_env}"
            "PYTHONPATH=${_python_prefix}" "PATH=$ENV{SystemRoot}/System32;$ENV{SystemRoot}"
            "${_python}" -s -B "${_work}/ma_python.py" "${_python_prefix}" "${_work}/runtime-data.sha256" ${_mode}
            "${_work}/joint/chain.joint.txt"
        WORKING_DIRECTORY "${_work}" RESULT_VARIABLE _result OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    file(COPY_FILE "${_work}/probe.installed" "${_target}")
    file(WRITE "${_work}/run-probe-${_probe_name}.log" "${_out}${_err}")
    list(APPEND _probe_results "${_change} ${_relative} ${_mode}=${_result}")
    if(NOT _result STREQUAL _expected_exit)
        message(FATAL_ERROR "installed data probe (${_change} ${_relative}, ${_mode}): expected exit ${_expected_exit}, got ${_result}; "
                            "see ${_work}/run-probe-${_probe_name}.log\n${_out}${_err}")
    endif()
    string(FIND "${_out}${_err}" "${_target}" _named)
    if(_expected_exit STREQUAL "4" AND _named EQUAL -1)
        message(FATAL_ERROR "installed data probe (${_change} ${_relative}): the refusal does not name ${_target}\n${_out}${_err}")
    endif()
endforeach()
foreach(_file IN LISTS _runtime_files)
    check_file("${_expected}/${_file}" "${_python_aircraft}/${_file}" "${_work}")
endforeach()
message(STATUS "installed data: ${_runtime_count} files of the expected runtime inventory in both packages, byte for byte; "
               "probes ${_probe_results}; restored")
# The composition example (examples/python/ma_composition; M6) on the installed package alone, copied out of the source
# tree: run twice, its traces byte-identical, its package and library the installed ones; and its MA system under the
# installed training adapter (fsim.training; M7 core) through "same_step" and "next_step" autoreset, each generation's
# trace the same as the example's own direct run.
file(COPY "${SOURCE_DIR}/examples/python/ma_composition" DESTINATION "${_work}" PATTERN "__pycache__" EXCLUDE)
foreach(_run 1 2)
    execute_process(COMMAND "${CMAKE_COMMAND}" -E env --unset=FSIM_DLL_DIR --unset=PYTHONHOME "${_python_env}"
            "PYTHONPATH=${_python_prefix}" "PATH=$ENV{SystemRoot}/System32;$ENV{SystemRoot}"
            "${_python}" -s -B -m ma_composition.run --trace "${_work}/composition-${_run}.json"
        WORKING_DIRECTORY "${_work}" RESULT_VARIABLE _result OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    file(WRITE "${_work}/run-composition-${_run}.log" "${_out}${_err}")
    if(NOT _result STREQUAL "0")
        message(FATAL_ERROR "installed composition example (run ${_run}): expected 0, got ${_result}\n${_out}${_err}")
    endif()
endforeach()
file(SHA256 "${_work}/composition-1.json" _first)
file(SHA256 "${_work}/composition-2.json" _second)
if(NOT _first STREQUAL _second)
    message(FATAL_ERROR "installed composition example: two runs with identical inputs gave different traces")
endif()
file(READ "${_work}/composition-1.json" _trace)
string(JSON _package GET "${_trace}" fsim)
string(JSON _library GET "${_trace}" library)
string(JSON _same GET "${_trace}" same_after_reset)
string(JSON _same_step GET "${_trace}" same_under_training same_step)
string(JSON _next_step GET "${_trace}" same_under_training next_step)
cmake_path(IS_PREFIX _python_prefix "${_package}" NORMALIZE _package_installed)
cmake_path(IS_PREFIX _python_prefix "${_library}" NORMALIZE _library_installed)
if(NOT _package_installed OR NOT _library_installed OR NOT _same OR NOT _same_step OR NOT _next_step)
    message(FATAL_ERROR "installed composition example: package ${_package}, library ${_library}, same after reset ${_same}, "
                        "same under the training adapter: same_step ${_same_step}, next_step ${_next_step}")
endif()
message(STATUS "installed composition example: two identical traces (${_first}), same after reset and under the training "
               "adapter (same_step, next_step), on ${_library}\n${_out}")
message(STATUS "installed MA consumers passed in C++, C99 and Python; provenance: ${_work}/provenance.sha256")
