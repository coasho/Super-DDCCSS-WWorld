#pragma once

// Recording of a world to a file and playback (design 9.5 "Recorder"): the writer and the reader of the format
// contracts/recording/RecordingFormat.h defines - its records, magic, version and frozen strides.

#include "contracts/recording/RecordingFormat.h"

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace fsim::ipc {

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
