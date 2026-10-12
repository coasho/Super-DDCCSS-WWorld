# cmake -DSRC=<repo>/aircraft -DDEST=<share/flightsim/aircraft> -P StageDesigns.cmake
#
# Copies what tools/hangar builds for each design - aircraft/<name>/<name>.xml,
# its Engines/ and the <name>.glb model with its manifest - and its payload fit
# (payloads.json, ADR-30), radar cross section (rcs.bin, MS-3), and damage data
# (<name>.damage.json and pieces/, MS-7g) to DEST/<name>
# (and models.txt, the viewer's stand-ins for stock aircraft, to DEST),
# where io::AssetResolver::findAircraft finds it in a package. A store's design
# (a missile, a bomb, a tank, a pod, cargo: hangar/storestages.py) has its
# flight model apart, in aircraft/stores/<name> - or is JSBSim's own - and is
# staged for its model and its table. The design's own files (the .toml, out/)
# stay behind. Run at build or install time, so a new design needs no
# reconfigure.
if(NOT SRC OR NOT DEST)
    message(FATAL_ERROR "StageDesigns.cmake: SRC and DEST are required")
endif()
file(GLOB _designs LIST_DIRECTORIES true "${SRC}/*")
set(_n 0)
set(_stores 0)
foreach(_d IN LISTS _designs)
    get_filename_component(_name "${_d}" NAME)
    if(NOT EXISTS "${_d}/${_name}.xml" AND NOT EXISTS "${_d}/${_name}.glb")
        continue()
    endif()
    set(_to "${DEST}/${_name}")
    file(MAKE_DIRECTORY "${_to}")
    if(EXISTS "${_d}/${_name}.xml")
        file(COPY "${_d}/${_name}.xml" DESTINATION "${_to}")
        math(EXPR _n "${_n} + 1")
    else()
        math(EXPR _stores "${_stores} + 1")
    endif()
    foreach(_dir Engines pieces)
        if(IS_DIRECTORY "${_d}/${_dir}")
            file(COPY "${_d}/${_dir}" DESTINATION "${_to}")
        endif()
    endforeach()
    foreach(_f "${_name}.glb" "${_name}.glb.manifest" "payloads.json" "rcs.bin" "${_name}.damage.json")
        if(EXISTS "${_d}/${_f}")
            file(COPY "${_d}/${_f}" DESTINATION "${_to}")
        endif()
    endforeach()
endforeach()
# released stores' own flight models (hangar storemodels: aircraft/stores/<name>/<name>.xml)
if(IS_DIRECTORY "${SRC}/stores")
    file(COPY "${SRC}/stores" DESTINATION "${DEST}")
endif()
# the viewer's stand-ins for stock aircraft (fsim hangar register)
if(EXISTS "${SRC}/models.txt")
    file(COPY "${SRC}/models.txt" DESTINATION "${DEST}")
endif()
message(STATUS "designed aircraft: ${_n} staged in ${DEST} (and ${_stores} stores' models)")
