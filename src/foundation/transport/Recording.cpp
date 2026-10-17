#include "foundation/transport/Recording.h"

#include "foundation/core/Log.h"

#include <algorithm>
#include <cstring>
#include <type_traits>
#include <utility>

namespace fsim::ipc {

namespace {
// (the format's magic, version history and records: contracts/recording/RecordingFormat.h)
constexpr const char (&kRecMagic)[5] = kRecordingMagic;
constexpr std::uint8_t kVersion = kRecordingVersion;
constexpr std::size_t kStateV1 = offsetof(sim::VehicleState, engineRpm);
static_assert(offsetof(VehicleSample, state) == 0, "a sample starts with its state");
/// A sample's bytes before version 6: all but the `derived` values added since (they are its last).
constexpr std::size_t kSampleV5 = offsetof(VehicleSample, derived) + kDerivedCountV7 * sizeof(double);
static_assert(kSampleV5 + (kDerivedCount - kDerivedCountV7) * sizeof(double) == sizeof(VehicleSample), "a sample ends with its derived values");

/// A version 2 to 6 sample: as it is now, less the `derived` values added since it (they are its last).
bool readSampleShort(std::FILE* f, VehicleSample& sample, std::uint8_t version) {
    sample = VehicleSample{};
    const std::size_t size = offsetof(VehicleSample, derived) + (version < 6 ? kDerivedCountV7 : kDerivedCountV8) * sizeof(double);
    return std::fread(static_cast<void*>(&sample), 1, size, f) == size;
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

using SnapshotHead = RecordingSnapshotHead;

// What a recording holds is its records' members alone (HC-01). A struct's padding - bytes no member occupies - is
// unspecified: written as a caller's struct or a local held it, equal worlds gave unequal files and carried unrelated
// memory into them. Each record is written as its own bytes with its padding zero; no size, offset, version or member
// changes, and a reader is untouched. The gaps are where the frozen layouts have them (tests/contracts/test_state_layout.cpp
// holds every member's offset): each is the span from one member's end to the next member's start.
using Gap = std::pair<std::size_t, std::size_t>; // offset, size

constexpr Gap between(std::size_t end, std::size_t next) { return {end, next - end}; }

constexpr Gap kHeaderGaps[] = {
    between(offsetof(RecordingHeader, capacity) + sizeof(std::uint32_t), offsetof(RecordingHeader, dt)),
    between(offsetof(RecordingHeader, name) + sizeof(RecordingHeader::name), sizeof(RecordingHeader)),
};
constexpr Gap kRowGaps[] = {
    between(offsetof(RecordedVehicle, id) + sizeof(std::uint32_t), offsetof(RecordedVehicle, generation)),
    between(offsetof(RecordedVehicle, model) + sizeof(RecordedVehicle::model), offsetof(RecordedVehicle, initialLatitudeDeg)),
};
constexpr Gap kHeadGaps[] = {
    between(offsetof(SnapshotHead, count) + sizeof(std::uint32_t), sizeof(SnapshotHead)),
};
// a sample's: its state's two (the layout contract's kVehicleStatePadding, shared with the world publisher); its inputs
// and derived values are doubles that follow the state and each other with none between
constexpr Gap kSampleGaps[] = {
    {kVehicleStatePadding[0].offset, kVehicleStatePadding[0].size},
    {kVehicleStatePadding[1].offset, kVehicleStatePadding[1].size},
};
static_assert(sizeof(RecordingHeader) == 88 && kHeaderGaps[0].first == 4 && kHeaderGaps[0].second == 4 &&
                  kHeaderGaps[1].first == 84 && kHeaderGaps[1].second == 4, "the frozen header's padding");
static_assert(sizeof(RecordedVehicle) == 440 && kRowGaps[0].first == 4 && kRowGaps[0].second == 4 && kRowGaps[1].first == 402 &&
                  kRowGaps[1].second == 6, "the frozen table row's padding");
static_assert(sizeof(SnapshotHead) == 16 && kHeadGaps[0].first == 12 && kHeadGaps[0].second == 4, "the snapshot head's padding");
static_assert(offsetof(VehicleSample, state) == 0 && kSampleGaps[0].first == 391 && kSampleGaps[0].second == 1 &&
                  kSampleGaps[1].first == 604 && kSampleGaps[1].second == 4, "the frozen sample state's padding");
static_assert(offsetof(VehicleSample, inputs) == sizeof(sim::VehicleState) &&
                  offsetof(VehicleSample, derived) == offsetof(VehicleSample, inputs) + sizeof(sim::ControlInputs) &&
                  sizeof(VehicleSample) == offsetof(VehicleSample, derived) + sizeof(VehicleSample::derived),
              "a sample's state, inputs and derived values follow each other with no padding");

/// `value` as a record: its bytes, its padding (`gaps`) zero. A record without padding (stores, environment, events)
/// is written with none.
template <class T, std::size_t N>
void writeRecord(std::FILE* f, const T& value, const Gap (&gaps)[N]) {
    static_assert(std::is_trivially_copyable_v<T>, "a record is plain data");
    unsigned char bytes[sizeof(T)];
    std::memcpy(bytes, &value, sizeof(T));
    for (const auto& [offset, size] : gaps) std::memset(bytes + offset, 0, size);
    std::fwrite(bytes, sizeof(T), 1, f);
}
constexpr Gap kNoGaps[1] = {{0, 0}};
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
    writeRecord(file_, header, kHeaderGaps);
    bytes_ = 6 + sizeof(header);
    LOG_INFO("ipc") << "recording to " << path.string();
    return true;
}

void RecordWriter::writeVehicle(std::uint32_t slot, const RecordedVehicle& row) {
    if (!file_) return;
    const std::uint8_t kind = 'T';
    std::fwrite(&kind, 1, 1, file_);
    std::fwrite(&slot, sizeof(slot), 1, file_);
    writeRecord(file_, row, kRowGaps);
    bytes_ += 1 + sizeof(slot) + sizeof(row);
}

void RecordWriter::writeSnapshot(double simTime, const std::vector<std::pair<std::uint32_t, VehicleSample>>& samples) {
    if (!file_) return;
    const std::uint8_t kind = 'S';
    const SnapshotHead head{simTime, static_cast<std::uint32_t>(samples.size())};
    std::fwrite(&kind, 1, 1, file_);
    writeRecord(file_, head, kHeadGaps);
    for (const auto& [slot, sample] : samples) {
        std::fwrite(&slot, sizeof(slot), 1, file_);
        writeRecord(file_, sample, kSampleGaps);
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
    for (const MissionStore& store : stores) writeRecord(file_, store, kNoGaps);
    bytes_ += 1 + 2 * sizeof(std::uint32_t) + count * sizeof(MissionStore);
}

void RecordWriter::writeEnvironment(const sim::EnvironmentState& environment) {
    if (!file_) return;
    const std::uint8_t kind = 'E';
    const auto size = static_cast<std::uint32_t>(sizeof(environment));
    std::fwrite(&kind, 1, 1, file_);
    std::fwrite(&size, sizeof(size), 1, file_);
    writeRecord(file_, environment, kNoGaps);
    bytes_ += 1 + sizeof(size) + sizeof(environment);
}

void RecordWriter::writeEvent(const EngagementEvent& event) {
    if (!file_) return;
    const std::uint8_t kind = 'V';
    const auto size = static_cast<std::uint32_t>(sizeof(event));
    std::fwrite(&kind, 1, 1, file_);
    std::fwrite(&size, sizeof(size), 1, file_);
    writeRecord(file_, event, kNoGaps);
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
                                  (version == 1 ? readSampleV1(f, sample) : version < kVersion ? readSampleShort(f, sample, version) : std::fread(&sample, sizeof(sample), 1, f) == 1);
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
