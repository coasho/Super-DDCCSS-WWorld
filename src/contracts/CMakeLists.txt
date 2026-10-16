# contracts: neutral definitions both sides of a boundary include - wire and recording layouts, stream batches, native
# construction values (ARCH-02; docs/architecture/design/boundaries.md). Header-only: standard library and the public
# value headers (include/fsim) only; no registry, World, OS service, graphics or domain implementation. The sources are
# listed for ownership and IDEs; nothing is compiled.
add_library(fsim_contracts INTERFACE
    world_stream/WorldLayout.h
    world_stream/SnapshotBatch.h
    vision_stream/VisionLayout.h
    recording/RecordingFormat.h
    simulation/WorldOptions.h)
add_library(fsim::contracts ALIAS fsim_contracts)

target_include_directories(fsim_contracts INTERFACE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/include)
