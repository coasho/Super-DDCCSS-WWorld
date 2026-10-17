# The include-view check's own refusals (cmake/CheckIncludeView.cmake). Run as a test:
#
#   cmake -DCHECK=<build>/include-view-checks/<target>.cmake -P TestCheckIncludeView.cmake
#
# CHECK is a generated, passing check for a target with a declared provider (fsim_ms; its provider is core). Each case
# writes a mutated copy and runs the check on it. The unmutated check and one with a permitted (non-ancestor)
# repository root pass. Each refusal must fail with its own reason:
#   - the src root or an ancestor of it, in every spelling: src, src/., the repository root, src/.., the repository's
#     parent (and, on Windows, src in another case) - refused by the root guard;
#   - a copy of src's headers outside the repository's ancestry, as a src root and as a repository-root layout - which
#     no path comparison recognizes, so the compile probes themselves must reject it;
#   - the provider's view removed; a stale probe; a compiler whose probe command is not established.
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
    math(EXPR _c "${_count} + 1")
    set(_count ${_c} PARENT_SCOPE)
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
set(_count 0)

# set(INCLUDES ...) rewritten: <list>
function(with_includes out list)
    string(REGEX REPLACE "set\\(INCLUDES \\[==\\[[^]]*\\]==\\]\\)" "set(INCLUDES [==[${list}]==])" _t "${_original}")
    set(${out} "${_t}" PARENT_SCOPE)
endfunction()

list(FILTER INCLUDES EXCLUDE REGEX "/include-views/fsim_core$")
set(_without_core "${INCLUDES}")
include("${CHECK}")

run_case(unmutated PASS "${_original}")
# a repository directory that is no ancestor of src stays permitted
with_includes(_t "${INCLUDES};${SOURCE_DIR}/third_party/ntia")
run_case(permitted_vendor_root PASS "${_t}")
# the src root and its ancestors, however spelled, are refused before any probe runs
set(_whole "holds the whole src tree \\(the src root or an ancestor of it\\)")
set(_ancestors
    shared_src_root "${SOURCE_DIR}/src"
    normalized_src_root "${SOURCE_DIR}/src/."
    repository_root "${SOURCE_DIR}"
    src_parent "${SOURCE_DIR}/src/.."
    repository_parent "${SOURCE_DIR}/..")
if(CMAKE_HOST_WIN32)
    string(TOUPPER "${SOURCE_DIR}/src" _upper)
    list(APPEND _ancestors src_root_in_other_case "${_upper}")
endif()
while(_ancestors)
    list(POP_FRONT _ancestors _name _root)
    with_includes(_t "${INCLUDES};${_root}")
    run_case(${_name} "${_whole}" "${_t}")
endwhile()
# an equivalent tree no string comparison recognizes - a copy of src's headers outside the repository's ancestry -
# is rejected by the compile probes themselves, in either spelling
file(COPY "${SOURCE_DIR}/src/" DESTINATION "${_cases}/copy/src" FILES_MATCHING PATTERN "*.h")
with_includes(_t "${INCLUDES};${_cases}/copy/src")
run_case(copied_src_tree "forbidden simulation/[^ ]+ is visible to ${TARGET}: it compiled" "${_t}")
with_includes(_t "${INCLUDES};${_cases}/copy")
run_case(copied_repository_layout "forbidden src/simulation/[^ ]+ is visible to ${TARGET}: it compiled" "${_t}")
with_includes(_t "${_without_core}")
run_case(provider_view_missing "allowed foundation/core/Units\\.h does not compile" "${_t}")
string(REPLACE "set(FORBIDDEN [==[" "set(FORBIDDEN [==[simulation/flight/NoSuchHeader.h;" _t "${_original}")
run_case(stale_probe "NoSuchHeader\\.h does not exist; the fixture is stale" "${_t}")
string(REPLACE "set(STYLE GNU)" "set(STYLE MSVC)" _t "${_original}")
run_case(unsupported_compiler "support GNU-style compilers only" "${_t}")

if(_failures)
    message(FATAL_ERROR "include view checker:${_failures}")
endif()
message(STATUS "include view checker: ${_count} cases hold")
