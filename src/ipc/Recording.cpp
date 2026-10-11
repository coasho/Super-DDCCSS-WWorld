#include "ipc/Recording.h"

#include "core/Log.h"

#include <algorithm>
#include <cstring>

namespace fsim::ipc {

namespace {
constexpr char kRecMagic[5] = {'F', 'S', 'R', 'E', 'C'};
// 2: VehicleState grew by the engines' and moving parts' state; a version 1
// file still reads, its samples' state the prefix the fields were appended to.
// 3: the vehicles' stores ('M' records); an older file has none
// 4: the world's environment ('E' records); an older file has none
// 5: engagement events ('V' records: MS-7h); an older file has none
// 6: a sample's CG from its model's origin (two more `derived` values, at its end); an older file's samples are that
//    much shorter and theirs 0 - drawn as they were, each model's origin at its CG
constexpr std::uint8_t kVersion = 6;
constexpr std::size_t kStateV1 = offsetof(sim::VehicleState, engineRpm);
static_assert(offsetof(VehicleSample, state) == 0, "a sample starts with its state");
/// A sample's bytes before version 6: all but the `derived` values added since (they are its last).
constexpr std::size_t kSampleV5 = offsetof(VehicleSample, derived) + kDerivedCountV7 * sizeof(double);
static_assert(kSampleV5 + (kDerivedCount - kDerivedCountV7) * sizeof(double) == sizeof(VehicleSample), "a sample ends with its derived values");

/// A version 2 to 5 sample: as it is now, less the values at its end.
bool readSampleV5(std::FILE* f, VehicleSample& sample) {
    sample = VehicleSample{};
    return std::fread(static_cast<void*>(&sample), 1, kSampleV5, f) == kSampleV5;
}

/// A version 1 sample: the old state, then the rest of the sample as it was.
bool readSampleV1(std::FILE* f, VehicleSample& sample) {
    sample = VehicleSample{};
    constexpr std::size_t rest = kSampleV5 - offsetof(VehicleSample, inputs);
    constexpr std::size_t oldInputs = (kStateV1 + alignof(sim::ControlInputs) - 1) / alignof(sim::ControlInputs) *
                                      alignof(sim::ControlInputs);
    unsigned char buffer[sizeof(VehicleSample)];
    const std::size_t size = oldInputs + rest;
    if (std::fread(buffer, 1, size, f) != size) return false;
    std::memcpy(static_cast<void*>(&sample.state), buffer, kStateV1); // the fields appended since keep their defaults
    std::memcpy(reinterpret_cast<unsigned char*>(&sample) + offsetof(VehicleSample, inputs), buffer + oldInputs, rest);
    return true;
}

struct SnapshotHead {
    double simTime;
    std::uint32_t count;
};
} // namespace

RecordWriter::~RecordWriter() { close(); }

bool RecordWriter::open(const std::filesystem::path& path, const RecordingHeader& header) {
    close();
#ifdef _WIN32
    file_ = _wfopen(path.wstring().c_str(), L"wb");
#else
    file_ = std::fopen(path.string().c_str(), "wb");
#endif
    if (!file_) {
        LOG_ERROR("ipc") << "cannot write recording " << path.string();
        return false;
    }
    std::fwrite(kRecMagic, 1, 5, file_);
    std::fwrite(&kVersion, 1, 1, file_);
    std::fwrite(&header, sizeof(header), 1, file_);
    bytes_ = 6 + sizeof(header);
    LOG_INFO("ipc") << "recording to " << path.string();
    return true;
}

void RecordWriter::writeVehicle(std::uint32_t slot, const RecordedVehicle& row) {
    if (!file_) return;
    const std::uint8_t kind = 'T';
    std::fwrite(&kind, 1, 1, file_);
    std::fwrite(&slot, sizeof(slot), 1, file_);
    std::fwrite(&row, sizeof(row), 1, file_);
    bytes_ += 1 + sizeof(slot) + sizeof(row);
}

void RecordWriter::writeSnapshot(double simTime, const std::vector<std::pair<std::uint32_t, VehicleSample>>& samples) {
    if (!file_) return;
    const std::uint8_t kind = 'S';
    const SnapshotHead head{simTime, static_cast<std::uint32_t>(samples.size())};
    std::fwrite(&kind, 1, 1, file_);
    std::fwrite(&head, sizeof(head), 1, file_);
    for (const auto& [slot, sample] : samples) {
        std::fwrite(&slot, sizeof(slot), 1, file_);
        std::fwrite(&sample, sizeof(sample), 1, file_);
    }
    bytes_ += 1 + sizeof(head) + samples.size() * (sizeof(std::uint32_t) + sizeof(VehicleSample));
}

void RecordWriter::writeStores(std::uint32_t slot, const std::vector<MissionStore>& stores) {
    if (!file_) return;
    const std::uint8_t kind = 'M';
    const auto count = static_cast<std::uint32_t>(stores.size());
    std::fwrite(&kind, 1, 1, file_);
    std::fwrite(&slot, sizeof(slot), 1, file_);
    std::fwrite(&count, sizeof(count), 1, file_);
    if (count) std::fwrite(stores.data(), sizeof(MissionStore), count, file_);
    bytes_ += 1 + 2 * sizeof(std::uint32_t) + count * sizeof(MissionStore);
}

void RecordWriter::writeEnvironment(const sim::EnvironmentState& environment) {
    if (!file_) return;
    const std::uint8_t kind = 'E';
    const auto size = static_cast<std::uint32_t>(sizeof(environment));
    std::fwrite(&kind, 1, 1, file_);
    std::fwrite(&size, sizeof(size), 1, file_);
    std::fwrite(&environment, sizeof(environment), 1, file_);
    bytes_ += 1 + sizeof(size) + sizeof(environment);
}

void RecordWriter::writeEvent(const EngagementEvent& event) {
    if (!file_) return;
    const std::uint8_t kind = 'V';
    const auto size = static_cast<std::uint32_t>(sizeof(event));
    std::fwrite(&kind, 1, 1, file_);
    std::fwrite(&size, sizeof(size), 1, file_);
    std::fwrite(&event, sizeof(event), 1, file_);
    bytes_ += 1 + sizeof(size) + sizeof(event);
}

void RecordWriter::close() {
    if (file_) std::fclose(file_);
    file_ = nullptr;
}

bool Recording::load(const std::filesystem::path& path, std::string* error) {
    frames_.clear();
#ifdef _WIN32
    std::FILE* f = _wfopen(path.wstring().c_str(), L"rb");
#else
    std::FILE* f = std::fopen(path.string().c_str(), "rb");
#endif
    if (!f) {
        if (error) *error = "cannot open " + path.string();
        return false;
    }
    char magic[5];
    std::uint8_t version = 0;
    if (std::fread(magic, 1, 5, f) != 5 || std::memcmp(magic, kRecMagic, 5) != 0 || std::fread(&version, 1, 1, f) != 1 ||
        version < 1 || version > kVersion ||
        std::fread(&header_, sizeof(header_), 1, f) != 1) {
        std::fclose(f);
        if (error) *error = "not an fsim recording: " + path.string();
        return false;
    }
    Frame pending; // table changes accumulate until the next snapshot
    for (;;) {
        std::uint8_t kind = 0;
        if (std::fread(&kind, 1, 1, f) != 1) break;
        if (kind == 'T') {
            std::uint32_t slot = 0;
            RecordedVehicle row;
            if (std::fread(&slot, sizeof(slot), 1, f) != 1 || std::fread(&row, sizeof(row), 1, f) != 1) break;
            pending.tableChanges.emplace_back(slot, row);
        } else if (kind == 'M') {
            std::uint32_t slot = 0, count = 0;
            if (std::fread(&slot, sizeof(slot), 1, f) != 1 || std::fread(&count, sizeof(count), 1, f) != 1 || count > kMissionStores) break;
            std::vector<MissionStore> stores(count);
            if (count && std::fread(stores.data(), sizeof(MissionStore), count, f) != count) break;
            pending.storeChanges.emplace_back(slot, std::move(stores));
        } else if (kind == 'E') {
            // (as much of it as this build knows; a later build's longer state read in part, the rest skipped)
            std::uint32_t size = 0;
            if (std::fread(&size, sizeof(size), 1, f) != 1 || size > (1u << 20)) break;
            sim::EnvironmentState env;
            const std::size_t known = std::min<std::size_t>(size, sizeof(env));
            if (std::fread(&env, 1, known, f) != known) break;
            if (size > known && std::fseek(f, static_cast<long>(size - known), SEEK_CUR) != 0) break;
            pending.environment = env;
        } else if (kind == 'V') {
            std::uint32_t size = 0;
            if (std::fread(&size, sizeof(size), 1, f) != 1 || size > (1u << 16)) break;
            EngagementEvent event;
            const std::size_t known = std::min<std::size_t>(size, sizeof(event));
            if (std::fread(&event, 1, known, f) != known) break;
            if (size > known && std::fseek(f, static_cast<long>(size - known), SEEK_CUR) != 0) break;
            pending.events.push_back(event);
        } else if (kind == 'S') {
            SnapshotHead head{};
            if (std::fread(&head, sizeof(head), 1, f) != 1) break;
            Frame frame;
            frame.simTime = head.simTime;
            frame.tableChanges.swap(pending.tableChanges);
            frame.storeChanges.swap(pending.storeChanges);
            frame.environment = pending.environment;
            pending.environment.reset();
            frame.events.swap(pending.events);
            frame.samples.resize(head.count);
            bool ok = true;
            for (auto& [slot, sample] : frame.samples) {
                const bool read = std::fread(&slot, sizeof(slot), 1, f) == 1 &&
                                  (version == 1 ? readSampleV1(f, sample) : version < 6 ? readSampleV5(f, sample) : std::fread(&sample, sizeof(sample), 1, f) == 1);
                if (!read) { ok = false; break; }
            }
            if (!ok) break;
            frames_.push_back(std::move(frame));
        } else {
            break; // unknown record: stop at what we have
        }
    }
    std::fclose(f);
    if (!pending.tableChanges.empty() || !pending.storeChanges.empty() || pending.environment || !pending.events.empty()) {
        if (frames_.empty()) frames_.push_back(Frame{});
        if (pending.environment) frames_.back().environment = pending.environment;
        auto& events = frames_.back().events;
        events.insert(events.end(), pending.events.begin(), pending.events.end());
        auto& last = frames_.back().tableChanges;
        last.insert(last.end(), pending.tableChanges.begin(), pending.tableChanges.end());
        auto& stores = frames_.back().storeChanges;
        for (auto& s : pending.storeChanges) stores.push_back(std::move(s));
    }
    LOG_INFO("ipc") << "recording " << path.string() << ": " << frames_.size() << " frames, " << duration() << " s";
    return !frames_.empty();
}

} // namespace fsim::ipc
