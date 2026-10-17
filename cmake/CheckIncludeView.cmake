# Compile-level include-view check (ARCH-03 task 5; cmake/IncludeViews.cmake). Run as a test:
#
#   cmake -DCHECK=<build>/include-view-checks/<target>.cmake -P CheckIncludeView.cmake
#
# CHECK is generated at configure time (tests/CMakeLists.txt, fsim_include_view_check) and sets TARGET, COMPILER,
# STYLE (GNU; another compiler is refused), SOURCE_DIR, WORK, the target's own compile environment (INCLUDES, DEFINES,
# OPTIONS) and the probes (ALLOWED, FORBIDDEN). Each probe is a translation unit holding one #include, compiled
# syntax-only with the real compiler in that environment:
#   - every ALLOWED header (the target's own, its declared providers') must compile;
#   - every FORBIDDEN header (an unrelated private module's) must fail, and for that reason alone: the diagnostic names
#     the header as not found. The same probe then compiles once the shared src root is added back, so the header is
#     sound and only its visibility differs - the configuration before narrowing would fail this check.
if(NOT CHECK OR NOT EXISTS "${CHECK}")
    message(FATAL_ERROR "CheckIncludeView: CHECK=<generated check file> is required")
endif()
include("${CHECK}")
foreach(_v TARGET COMPILER STYLE SOURCE_DIR WORK ALLOWED FORBIDDEN)
    if(NOT ${_v})
        message(FATAL_ERROR "CheckIncludeView: ${CHECK} does not set ${_v}")
    endif()
endforeach()

# The supported presets (UCRT64) use GCC; another compiler's probe command and diagnostics are not established here.
if(NOT STYLE STREQUAL "GNU")
    message(FATAL_ERROR "${TARGET}: include view probes support GNU-style compilers only (this build: ${STYLE})")
endif()
set(_base -std=c++17 -fsyntax-only)
set(_inc -I)
set(_def -D)
foreach(_d IN LISTS INCLUDES)
    list(APPEND _base "${_inc}${_d}")
endforeach()
foreach(_d IN LISTS DEFINES)
    list(APPEND _base "${_def}${_d}")
endforeach()
list(APPEND _base ${OPTIONS})
if("${SOURCE_DIR}/src" IN_LIST INCLUDES)
    message(FATAL_ERROR "${TARGET}: its include environment holds the shared src root ${SOURCE_DIR}/src")
endif()

file(REMOVE_RECURSE "${WORK}")
file(MAKE_DIRECTORY "${WORK}")
set(_n 0)
# probe(<header> <result-var> <output-var> [extra flags...])
function(probe header result output)
    math(EXPR _k "${_n} + 1")
    set(_n ${_k} PARENT_SCOPE)
    set(_tu "${WORK}/probe_${_k}.cpp")
    file(WRITE "${_tu}" "#include \"${header}\"\n")
    execute_process(COMMAND "${COMPILER}" ${_base} ${ARGN} "${_tu}"
        WORKING_DIRECTORY "${WORK}" RESULT_VARIABLE _r OUTPUT_VARIABLE _o ERROR_VARIABLE _o)
    set(${result} "${_r}" PARENT_SCOPE)
    set(${output} "${_o}" PARENT_SCOPE)
endfunction()

function(require_source header)
    if(NOT EXISTS "${SOURCE_DIR}/src/${header}" AND NOT EXISTS "${SOURCE_DIR}/include/${header}")
        message(FATAL_ERROR "${TARGET}: probe header ${header} does not exist; the fixture is stale")
    endif()
endfunction()

set(_failures "")
foreach(_h IN LISTS ALLOWED)
    require_source("${_h}")
    probe("${_h}" _r _o)
    if(_r EQUAL 0)
        message(STATUS "${TARGET}: allowed   ${_h} - compiles")
    else()
        list(APPEND _failures "allowed ${_h} does not compile in ${TARGET}'s environment:\n${_o}")
    endif()
endforeach()
foreach(_h IN LISTS FORBIDDEN)
    require_source("${_h}")
    probe("${_h}" _r _o)
    string(REPLACE "." "\\." _pattern "${_h}")
    if(_r EQUAL 0)
        list(APPEND _failures "forbidden ${_h} is visible to ${TARGET}: it compiled")
        continue()
    elseif(NOT _o MATCHES "${_pattern}'?:? (No such file or directory|file not found)")
        list(APPEND _failures "forbidden ${_h} failed in ${TARGET} for another reason than not being found:\n${_o}")
        continue()
    endif()
    probe("${_h}" _rc _oc "${_inc}${SOURCE_DIR}/src")
    if(NOT _rc EQUAL 0)
        list(APPEND _failures "forbidden ${_h} does not compile in ${TARGET}'s environment even with the shared src root, "
             "so its probe proves nothing:\n${_oc}")
        continue()
    endif()
    message(STATUS "${TARGET}: forbidden ${_h} - not found (compiles with the shared src root)")
endforeach()

if(_failures)
    list(JOIN _failures "\n" _text)
    message(FATAL_ERROR "${TARGET}: include view check failed:\n${_text}")
endif()
list(LENGTH ALLOWED _na)
list(LENGTH FORBIDDEN _nf)
message(STATUS "${TARGET}: include view holds - ${_na} allowed compile, ${_nf} forbidden not found")
