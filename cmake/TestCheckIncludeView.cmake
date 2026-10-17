# The include-view check's own refusals (cmake/CheckIncludeView.cmake). Run as a test:
#
#   cmake -DCHECK=<build>/include-view-checks/<target>.cmake -P TestCheckIncludeView.cmake
#
# CHECK is a generated, passing check for a target with a declared provider (fsim_ms; its provider is core). Each case
# writes a mutated copy, runs the check on it and requires it to fail with the expected reason: the shared src root
# restored; an equivalent broader root the string guard does not catch, which only the compile probes can reject; the
# provider's view removed; a stale probe; a compiler whose probe command is not established. The unmutated check passes.
if(NOT CHECK OR NOT EXISTS "${CHECK}")
    message(FATAL_ERROR "TestCheckIncludeView: CHECK=<generated check file> is required")
endif()
include("${CHECK}")
set(_script "${CMAKE_CURRENT_LIST_DIR}/CheckIncludeView.cmake")
set(_cases "${WORK}-cases")
file(REMOVE_RECURSE "${_cases}")
file(MAKE_DIRECTORY "${_cases}")
file(READ "${CHECK}" _original)

set(_failures "")
# run_case(<name> <expected-regex-or-PASS> <check-text>)
function(run_case name expected text)
    set(_file "${_cases}/${name}.cmake")
    string(REPLACE "set(WORK [==[${WORK}]==])" "set(WORK [==[${_cases}/${name}]==])" text "${text}")
    file(WRITE "${_file}" "${text}")
    execute_process(COMMAND "${CMAKE_COMMAND}" "-DCHECK=${_file}" -P "${_script}"
        RESULT_VARIABLE _r OUTPUT_VARIABLE _o ERROR_VARIABLE _o)
    string(REGEX REPLACE "[ \t\r\n]+" " " _o "${_o}") # CMake wraps long messages
    if(expected STREQUAL "PASS")
        if(NOT _r EQUAL 0)
            set(_failures "${_failures}\n${name}: expected to pass:\n${_o}" PARENT_SCOPE)
            return()
        endif()
    elseif(_r EQUAL 0)
        set(_failures "${_failures}\n${name}: expected a failure matching '${expected}', but the check passed" PARENT_SCOPE)
        return()
    elseif(NOT _o MATCHES "${expected}")
        set(_failures "${_failures}\n${name}: failed, but not with '${expected}':\n${_o}" PARENT_SCOPE)
        return()
    endif()
    message(STATUS "${name}: ${expected}")
endfunction()

# set(INCLUDES ...) rewritten: <list>
function(with_includes out list)
    string(REGEX REPLACE "set\\(INCLUDES \\[==\\[[^]]*\\]==\\]\\)" "set(INCLUDES [==[${list}]==])" _t "${_original}")
    set(${out} "${_t}" PARENT_SCOPE)
endfunction()

list(FILTER INCLUDES EXCLUDE REGEX "/include-views/fsim_core$")
set(_without_core "${INCLUDES}")
include("${CHECK}")

run_case(unmutated PASS "${_original}")
with_includes(_t "${INCLUDES};${SOURCE_DIR}/src")
run_case(shared_src_root "holds the shared src root" "${_t}")
with_includes(_t "${INCLUDES};${SOURCE_DIR}/src/.")
run_case(broader_root "forbidden [^ ]+ is visible to ${TARGET}: it compiled" "${_t}")
with_includes(_t "${_without_core}")
run_case(provider_view_missing "allowed foundation/core/Units\\.h does not compile" "${_t}")
string(REPLACE "set(FORBIDDEN [==[" "set(FORBIDDEN [==[simulation/flight/NoSuchHeader.h;" _t "${_original}")
run_case(stale_probe "NoSuchHeader\\.h does not exist; the fixture is stale" "${_t}")
string(REPLACE "set(STYLE GNU)" "set(STYLE MSVC)" _t "${_original}")
run_case(unsupported_compiler "support GNU-style compilers only" "${_t}")

if(_failures)
    message(FATAL_ERROR "include view checker:${_failures}")
endif()
message(STATUS "include view checker: 6 cases hold")
