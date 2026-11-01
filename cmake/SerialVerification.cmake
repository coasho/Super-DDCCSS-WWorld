# Include at the end of each test-registration directory. Keep CMake 3.25
# support: cross-directory TEST property access requires newer CMake versions.
# Native CPU fixtures retain parallelism; scripts can configure/build their own
# fixtures, and render tests own the single GUI slot.
get_property(_fsim_slot_tests DIRECTORY PROPERTY TESTS)
foreach(_fsim_slot_test IN LISTS _fsim_slot_tests)
    # COMMAND is not a readable CMake TEST property. Conservatively serialize
    # static registrations, except the explicitly read-only fleet CPU shards.
    # Catch's generated native CPU registrations remain parallel; generated GPU
    # registrations receive RUN_SERIAL at catch_discover_tests itself.
    get_test_property("${_fsim_slot_test}" LABELS _fsim_slot_labels)
    if(NOT _fsim_slot_test MATCHES "^fleet: .*shard [0-9]+ of [0-9]+$" AND
       NOT "slot-read-only" IN_LIST _fsim_slot_labels)
        set_tests_properties("${_fsim_slot_test}" PROPERTIES RUN_SERIAL TRUE)
    endif()
endforeach()
