# Optional external OpenVSP 3.55.0 headless-build fixtures. Never included by
# the SDK build. Include this from upstream src/vsp/CMakeLists.txt explicitly.
if(NOT TARGET tritri OR NOT TARGET robust OR NOT TARGET Eigen3::Eigen
   OR NOT VSP_LIBRARIES_API_FIRST)
  message(FATAL_ERROR "Intersection fixtures require the external OpenVSP geometry/API build")
endif()

set(FSIM_OPENVSP_INTERSECTION_SOURCE "" CACHE FILEPATH
  "Optional explicit intersection source override; no patch is applied")
set(_fsim_intersection_override)
if(FSIM_OPENVSP_INTERSECTION_SOURCE)
  if(NOT EXISTS "${FSIM_OPENVSP_INTERSECTION_SOURCE}")
    message(FATAL_ERROR "Explicit intersection source override is missing")
  endif()
  file(SHA256 "${FSIM_OPENVSP_INTERSECTION_SOURCE}" _fsim_intersection_sha256)
  message(STATUS "Explicit intersection source SHA256: ${_fsim_intersection_sha256}")
  set(_fsim_intersection_override "${FSIM_OPENVSP_INTERSECTION_SOURCE}")
  set_source_files_properties("${FSIM_OPENVSP_INTERSECTION_SOURCE}" PROPERTIES LANGUAGE CXX)
endif()

add_executable(fsim_vsp_intersection_boundary
  "${CMAKE_CURRENT_LIST_DIR}/vsp_triangle_boundary.cpp" ${_fsim_intersection_override})
target_include_directories(fsim_vsp_intersection_boundary PRIVATE
  ${TRITRI_INCLUDE_DIR} ${ROBUST_INCLUDE_DIR} ${GTE_INCLUDE_DIR})
target_link_libraries(fsim_vsp_intersection_boundary Eigen3::Eigen ${VSP_LIBRARIES_API_FIRST})

add_executable(fsim_vsp_intersection_contact
  "${CMAKE_CURRENT_LIST_DIR}/vsp_triangle_contact.cpp" ${_fsim_intersection_override})
target_include_directories(fsim_vsp_intersection_contact PRIVATE
  ${TRITRI_INCLUDE_DIR} ${ROBUST_INCLUDE_DIR} ${GTE_INCLUDE_DIR})
target_link_libraries(fsim_vsp_intersection_contact PRIVATE tritri robust)

add_executable(fsim_vsp_intersection_nonfinite
  "${CMAKE_CURRENT_LIST_DIR}/vsp_triangle_nonfinite.cpp" ${_fsim_intersection_override})
target_include_directories(fsim_vsp_intersection_nonfinite PRIVATE
  ${TRITRI_INCLUDE_DIR} ${ROBUST_INCLUDE_DIR} ${GTE_INCLUDE_DIR})
target_link_libraries(fsim_vsp_intersection_nonfinite PRIVATE tritri robust)

add_test(NAME fsim_vsp_intersection_boundary COMMAND fsim_vsp_intersection_boundary)
add_test(NAME fsim_vsp_intersection_contact COMMAND fsim_vsp_intersection_contact)
add_test(NAME fsim_vsp_intersection_nonfinite COMMAND fsim_vsp_intersection_nonfinite)
set_tests_properties(fsim_vsp_intersection_boundary fsim_vsp_intersection_contact fsim_vsp_intersection_nonfinite
  PROPERTIES TIMEOUT 30 LABELS "offline-intersection"
  ENVIRONMENT "TMP=D:/ENV/tmp;TEMP=D:/ENV/tmp;TORCH_EXTENSIONS_DIR=D:/ENV/tmp;OMP_NUM_THREADS=2")
