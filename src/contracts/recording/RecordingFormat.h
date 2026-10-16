#pragma once

// The recording file format (design 9.5 "Recorder"): the same vehicle table rows and samples the shared-memory segment
// carries, appended at a fixed simulation-time interval. Format v1-v7 (little-endian):
//   "FSREC" u8 version (currently 7) | RecordingHeader | records...
//   record: u8 kind ('T' table row, 'S' snapshot, 'M' stores) followed by
//     'T': u32 slot, RecordedVehicle (plain table row without atomics)
//     'S': RecordingSnapshotHead{double simTime; u32 count}, count x {u32 slot, VehicleSample}
//          (version 6: a sample ends with its CG's place from its model's origin - two more `derived` values; an
//          older file's samples are that much shorter and read with those 0; version 7: four more, each engine's
//          fuel flow)
//     'M': u32 slot, u32 count, count x MissionStore - a vehicle's stores as they are from the next snapshot on
//          (version 3)
//     'E': u32 size, size bytes of sim::EnvironmentState - the world's environment (its air, wind, weather, clock) as
//          it is from the next snapshot on, written when it changes (version 4; a reader takes the fields it knows)
//     'V': u32 size, size bytes of EngagementEvent - an engagement's event (MS-7h: a detonation) as it happened,
//          before the next snapshot (version 5; a reader takes the fields it knows)
//
// The header, table rows and sample prefixes below are raw fixed-size records,
// not extensible public structs. Their existing sizes/offsets and the v1-v7
// sample strides are frozen (tests/test_state_layout.cpp). New state requires an
// explicitly versioned format and a reader that converts each supported old
// representation; changing sizeof a current struct cannot define that conversion.
//
// A neutral contract (src/contracts, ARCH-02): the writer and reader (ipc/Recording.h, foundation transport) and the
// readers above them include it. Its names and namespace are those the definitions had in src/ipc.

#include "contracts/world_stream/WorldLayout.h"

#include <cstdint>

namespace fsim::ipc {

inline constexpr char kRecordingMagic[5] = {'F', 'S', 'R', 'E', 'C'};
// 2: VehicleState grew by the engines' and moving parts' state; a version 1
// file still reads, its samples' state the prefix the fields were appended to.
// 3: the vehicles' stores ('M' records); an older file has none
// 4: the world's environment ('E' records); an older file has none
// 5: engagement events ('V' records: MS-7h); an older file has none
// 6: a sample's CG from its model's origin (two more `derived` values, at its end); an older file's samples are that
//    much shorter and theirs 0 - drawn as they were, each model's origin at its CG
// 7: each engine's fuel flow (four more `derived` values); an older file's are 0 - its trails laid as they were, from
//    the thrust
inline constexpr std::uint8_t kRecordingVersion = 7;

struct RecordingHeader {
    std::uint32_t capacity = 0;
    double dt = 0.0;
    std::int32_t frameSkip = 1;
    char name[kNameLength] = {};
};

/// Plain-data copy of a table row (no atomics) as stored in the file.
struct RecordedVehicle {
    std::uint32_t id = 0;
    std::uint64_t generation = 0;
    std::uint8_t alive = 0;
    std::uint8_t controlLevel = 0;
    char name[kNameLength] = {};
    char type[kTypeLength] = {};
    char model[kPathLength] = {};
    double initialLatitudeDeg = 0.0, initialLongitudeDeg = 0.0, initialAltitudeMslM = 0.0, initialHeadingDeg = 0.0;
};

/// What an 'S' record starts with: its simulation time and how many {slot, sample} pairs follow.
struct RecordingSnapshotHead {
    double simTime;
    std::uint32_t count;
};

} // namespace fsim::ipc
