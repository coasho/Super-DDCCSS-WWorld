#pragma once

// Recording of a world to a file and playback (design 9.5 "Recorder"): the
// same vehicle table rows and samples the shared-memory segment carries,
// appended at a fixed simulation-time interval. Format v1-v7 (little-endian):
//   "FSREC" u8 version (currently 7) | RecordingHeader | records...
//   record: u8 kind ('T' table row, 'S' snapshot, 'M' stores) followed by
//     'T': u32 slot, RecordedVehicle (plain table row without atomics)
//     'S': SnapshotHead{double simTime; u32 count}, count x {u32 slot, VehicleSample}
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

#include "ipc/WorldLayout.h"

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace fsim::ipc {

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

class RecordWriter {
public:
    RecordWriter() = default;
    ~RecordWriter();
    RecordWriter(const RecordWriter&) = delete;
    RecordWriter& operator=(const RecordWriter&) = delete;

    bool open(const std::filesystem::path& path, const RecordingHeader& header);
    bool isOpen() const noexcept { return file_ != nullptr; }
    void writeVehicle(std::uint32_t slot, const RecordedVehicle& row);
    /// `samples` are the alive slots only (slot index + sample).
    void writeSnapshot(double simTime, const std::vector<std::pair<std::uint32_t, VehicleSample>>& samples);
    /// A vehicle's stores (MS-7), station by station, whenever they change: in force from the next snapshot.
    void writeStores(std::uint32_t slot, const std::vector<MissionStore>& stores);
    /// The world's environment, whenever it changes: in force from the next snapshot.
    void writeEnvironment(const sim::EnvironmentState& environment);
    /// An engagement's event (MS-7h), as it happens: it belongs to the next snapshot.
    void writeEvent(const EngagementEvent& event);
    void close();
    std::uint64_t bytesWritten() const noexcept { return bytes_; }

private:
    std::FILE* file_ = nullptr;
    std::uint64_t bytes_ = 0;
};

/// Whole recording in memory, frames indexed by simulation time.
class Recording {
public:
    struct Frame {
        double simTime = 0.0;
        std::vector<std::pair<std::uint32_t, VehicleSample>> samples;
        /// Table changes that happened at or before this frame since the previous one.
        std::vector<std::pair<std::uint32_t, RecordedVehicle>> tableChanges;
        /// Vehicles' stores as they changed since the previous frame (each the whole list, station by station).
        std::vector<std::pair<std::uint32_t, std::vector<MissionStore>>> storeChanges;
        /// The environment, where it changed since the previous frame (a recording before version 4 has none: the
        /// viewer's defaults).
        std::optional<sim::EnvironmentState> environment;
        /// Engagement events since the previous frame (MS-7h; before version 5, none).
        std::vector<EngagementEvent> events;
    };

    bool load(const std::filesystem::path& path, std::string* error = nullptr);
    const RecordingHeader& header() const noexcept { return header_; }
    const std::vector<Frame>& frames() const noexcept { return frames_; }
    double duration() const noexcept { return frames_.empty() ? 0.0 : frames_.back().simTime - frames_.front().simTime; }

private:
    RecordingHeader header_;
    std::vector<Frame> frames_;
};

} // namespace fsim::ipc
