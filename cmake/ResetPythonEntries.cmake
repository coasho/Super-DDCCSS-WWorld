# Generated interpreter entries are valid only for the current configure.
# Leave the package data intact; only the launch entries need invalidation.
file(REMOVE
    "${CMAKE_BINARY_DIR}/python/fsim-python.json"
    "${CMAKE_BINARY_DIR}/python/fsim-python.cmd")
