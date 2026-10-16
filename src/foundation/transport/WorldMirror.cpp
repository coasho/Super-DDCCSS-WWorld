#include "foundation/transport/WorldMirror.h"

#include "foundation/core/Log.h"
#include "foundation/platform/Clock.h"

#include <cstring>

namespace fsim::ipc {

namespace {

std::string decode(const char* s, std::size_t capacity) { return std::string(s, ::strnlen(s, capacity)); }

} // namespace

WorldMirror::~WorldMirror() { close(); }

bool WorldMirror::open(const std::string& name) {
    close();
    const std::string mapped = std::string(kWorldPrefix) + name;
    // Map the header first to learn the capacity, then the whole segment.
    platform::SharedMemory probe;
    if (!probe.open(mapped, sizeof(WorldHeader))) return false;
    const auto* h = static_cast<const WorldHeader*>(probe.data());
    if (h->magic == 0) return false; // (being initialised: looked at again next time)
    if (h->magic != kMagic || h->layoutVersion != kLayoutVersion || h->capacity == 0) {
        LOG_WARN("ipc") << "world '" << name << "': incompatible segment (magic " << h->magic << ", layout " << h->layoutVersion << ")";
        return false;
    }
    const std::size_t total = static_cast<std::size_t>(h->totalSize);
    probe.close();
    if (!memory_.open(mapped, total)) return false;
    base_ = static_cast<const unsigned char*>(memory_.data());
    header_ = reinterpret_cast<const WorldHeader*>(base_);
    name_ = name;
    vehicles_.assign(header_->capacity, MirroredVehicle{});
    inputs_.assign(header_->capacity, sim::ControlInputs{});
    status_.assign(header_->capacity, FlightStatus{});
    tableGeneration_ = ~0ull;
    lastSequence_ = 0;
    eventsSeen_ = 0;
    incarnation_ = header_->incarnation.load(std::memory_order_acquire);
    const_cast<WorldHeader*>(header_)->readers.fetch_add(1, std::memory_order_relaxed);
    attached_ = true;
    LOG_INFO("ipc") << "mirroring world '" << name << "' from process " << header_->publisherPid << " (" << header_->capacity
                    << " slots, dt " << header_->dt << ", frame skip " << header_->frameSkip << ")";
    return true;
}

void WorldMirror::close() {
    if (attached_ && header_) const_cast<WorldHeader*>(header_)->readers.fetch_sub(1, std::memory_order_relaxed);
    attached_ = false;
    header_ = nullptr;
    base_ = nullptr;
    memory_.close();
    vehicles_.clear();
    inputs_.clear();
    status_.clear();
}

bool WorldMirror::publisherAlive() const noexcept {
    return header_ && header_->alive.load(std::memory_order_acquire) != 0 && platform::processAlive(header_->publisherPid);
}

bool WorldMirror::replaced() const noexcept {
    return header_ && (header_->magic != kMagic || header_->incarnation.load(std::memory_order_acquire) != incarnation_);
}

double WorldMirror::ageSeconds() const noexcept {
    if (!header_) return 1e9;
    const std::int64_t last = header_->lastPublishWallNs.load(std::memory_order_acquire);
    if (last == 0) return 1e9;
    return static_cast<double>(platform::Clock::nanoseconds() - last) * 1e-9;
}

const VehicleRecord* WorldMirror::row(std::uint32_t index) const noexcept {
    return reinterpret_cast<const VehicleRecord*>(base_ + header_->tableOffset + sizeof(VehicleRecord) * index);
}

const StateSlotHeader* WorldMirror::slot(std::uint32_t index) const noexcept {
    return reinterpret_cast<const StateSlotHeader*>(base_ + header_->slotOffset + header_->slotStride * index);
}

const VehicleSample* WorldMirror::samples(std::uint32_t index) const noexcept {
    return reinterpret_cast<const VehicleSample*>(base_ + header_->slotOffset + header_->slotStride * index + Layout::align(sizeof(StateSlotHeader)));
}

bool WorldMirror::pollTable() {
    if (!header_) return false;
    const std::uint64_t generation = header_->tableGeneration.load(std::memory_order_acquire);
    if (generation == tableGeneration_) return false;
    for (std::uint32_t i = 0; i < header_->capacity; ++i) {
        const VehicleRecord* r = row(i);
        VehicleRecord copy;
        for (int attempt = 0; attempt < 16; ++attempt) {
            const std::uint32_t s0 = r->seq.load(std::memory_order_acquire);
            if (s0 & 1u) continue;
            copy.id = r->id;
            copy.generation = r->generation;
            copy.alive = r->alive;
            copy.controlLevel = r->controlLevel;
            std::memcpy(copy.name, r->name, kNameLength);
            std::memcpy(copy.type, r->type, kTypeLength);
            std::memcpy(copy.model, r->model, kPathLength);
            copy.initialLatitudeDeg = r->initialLatitudeDeg;
            copy.initialLongitudeDeg = r->initialLongitudeDeg;
            copy.initialAltitudeMslM = r->initialAltitudeMslM;
            copy.initialHeadingDeg = r->initialHeadingDeg;
            std::atomic_thread_fence(std::memory_order_acquire);
            if (r->seq.load(std::memory_order_acquire) == s0) break;
        }
        MirroredVehicle& v = vehicles_[i];
        v.slot = i;
        v.id = copy.id;
        v.generation = copy.generation;
        v.alive = copy.alive != 0;
        v.controlLevel = copy.controlLevel;
        v.name = decode(copy.name, kNameLength);
        v.type = decode(copy.type, kTypeLength);
        v.model = decode(copy.model, kPathLength);
        v.initialLatitudeDeg = copy.initialLatitudeDeg;
        v.initialLongitudeDeg = copy.initialLongitudeDeg;
        v.initialAltitudeMslM = copy.initialAltitudeMslM;
        v.initialHeadingDeg = copy.initialHeadingDeg;
    }
    tableGeneration_ = generation;
    return true;
}

std::size_t WorldMirror::pollEvents(std::vector<EngagementEvent>& out) {
    if (!header_ || header_->engagementOffset == 0) return 0;
    const auto* ring = reinterpret_cast<const EngagementRing*>(base_ + header_->engagementOffset);
    const std::uint64_t written = ring->written.load(std::memory_order_acquire);
    if (written < eventsSeen_) eventsSeen_ = 0; // (a world begun again)
    if (written == eventsSeen_) return 0;
    std::uint64_t from = std::max(eventsSeen_, written > kEngagementRing ? written - kEngagementRing : 0);
    const std::size_t before = out.size();
    for (std::uint64_t k = from; k < written; ++k) out.push_back(ring->events[k % kEngagementRing]);
    // (any the writer went round onto while they were copied: dropped)
    const std::uint64_t now = ring->written.load(std::memory_order_acquire);
    if (now > from + kEngagementRing) {
        const std::size_t lost = static_cast<std::size_t>(std::min<std::uint64_t>(now - kEngagementRing - from, written - from));
        out.erase(out.begin() + static_cast<std::ptrdiff_t>(before), out.begin() + static_cast<std::ptrdiff_t>(before + lost));
    }
    eventsSeen_ = written;
    return out.size() - before;
}

bool WorldMirror::pollSnapshot(sim::SnapshotBatch& out) {
    if (!header_) return false;
    if (header_->publishSequence.load(std::memory_order_acquire) <= lastSequence_) return false;
    const std::uint32_t n = header_->capacity;
    if (out.states.size() != n) out.states.resize(n);
    for (int attempt = 0; attempt < 8; ++attempt) {
        const std::uint32_t index = header_->latestSlot.load(std::memory_order_acquire);
        const StateSlotHeader* s = slot(index);
        const VehicleSample* v = samples(index);
        const std::uint32_t s0 = s->seq.load(std::memory_order_acquire);
        if (s0 & 1u) continue;
        const std::uint64_t sequence = s->sequence;
        const double simTime = s->simTime;
        const std::int64_t wallNs = s->wallNs;
        for (std::uint32_t i = 0; i < n; ++i) {
            out.states[i] = v[i].state;
            inputs_[i] = v[i].inputs;
            std::memcpy(status_[i].derived, v[i].derived, sizeof status_[i].derived);
        }
        std::atomic_thread_fence(std::memory_order_acquire);
        if (s->seq.load(std::memory_order_acquire) != s0) continue; // overwritten while copying: retry
        if (sequence <= lastSequence_) return false;
        out.sequence = sequence;
        out.simTime = simTime;
        out.wallNs = wallNs;
        lastSequence_ = sequence;
        lastWallNs_ = wallNs;
        return true;
    }
    return false;
}

bool WorldMirror::mission(std::uint32_t slot, MissionRecord& out, std::uint64_t known) const noexcept {
    if (!header_ || slot >= header_->capacity || header_->missionStride == 0) return false;
    const auto* m = reinterpret_cast<const MissionRecord*>(base_ + header_->missionOffset + header_->missionStride * slot);
    for (int attempt = 0; attempt < 8; ++attempt) {
        const std::uint32_t s0 = m->seq.load(std::memory_order_acquire);
        if (s0 & 1u) continue;
        if (m->revision == known) return false;
        std::memcpy(reinterpret_cast<char*>(&out) + sizeof(SeqLock), reinterpret_cast<const char*>(m) + sizeof(SeqLock),
                    sizeof(MissionRecord) - sizeof(SeqLock));
        std::atomic_thread_fence(std::memory_order_acquire);
        if (m->seq.load(std::memory_order_acquire) == s0) return true;
    }
    return false;
}

sim::EnvironmentState WorldMirror::environment() const noexcept {
    sim::EnvironmentState e;
    if (!header_) return e;
    for (int attempt = 0; attempt < 8; ++attempt) {
        const std::uint32_t s0 = header_->environmentSeq.load(std::memory_order_acquire);
        if (s0 & 1u) continue;
        e = header_->environment;
        std::atomic_thread_fence(std::memory_order_acquire);
        if (header_->environmentSeq.load(std::memory_order_acquire) == s0) break;
    }
    return e;
}

} // namespace fsim::ipc
