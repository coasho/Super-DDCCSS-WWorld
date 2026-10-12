# Script mode: cmake -DDIR=<dir> [-DKEEP=<name>] [-DBUILD_ROOT=<build>] -P CleanDist.cmake
#
# Empty a package directory before it is installed into again, except for one
# child: the viewer's maps/, gigabytes that install() copies only where they
# changed. Everything else goes, including what no install ever put there: a
# package that has been run in place collects whatever the run left behind -
# the NVIDIA driver writes "NVIDIA Corporation/umdlogs" into the working
# directory - and none of that belongs in what is shipped.
cmake_minimum_required(VERSION 3.25)
if(NOT IS_DIRECTORY "${DIR}")
    return()
endif()
if(BUILD_ROOT)
    # dist only owns direct component children of this build's dist directory.
    # Resolve existing paths as well: a symlink must not redirect cleanup into
    # an unrelated tree outside the supplied build directory.
    file(REAL_PATH "${BUILD_ROOT}" _build_root)
    file(REAL_PATH "${DIR}" _clean_dir)
    get_filename_component(_clean_parent "${_clean_dir}" DIRECTORY)
    if(NOT _clean_parent STREQUAL "${_build_root}/dist")
        message(FATAL_ERROR "CleanDist: refusing path outside owned build/dist: ${_clean_dir}")
    endif()
    get_filename_component(_component "${_clean_dir}" NAME)
    if(NOT _component MATCHES "^(sdk|tools|examples|viewer|python)$")
        message(FATAL_ERROR "CleanDist: refusing unknown component: ${_component}")
    endif()
endif()
file(GLOB _children LIST_DIRECTORIES true "${DIR}/*")
foreach(_child IN LISTS _children)
    get_filename_component(_name "${_child}" NAME)
    if(NOT _name STREQUAL "${KEEP}")
        file(REMOVE_RECURSE "${_child}")
    endif()
endforeach()
