#include "foundation/transport/DebrisSegmentV1.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <new>
#include <unordered_map>

namespace fsim::ipc {

namespace {

std::string segmentName(const std::string& world) { return "fsim.debris.v1." + world; }

std::size_t align64(std::size_t n) noexcept { return (n + 63u) & ~static_cast<std::size_t>(63u); }

std::size_t slotStride(std::uint32_t debrisCap, std::uint32_t vehicleCap) noexcept {
    return align64(sizeof(DebrisSlotV1)) + align64(sizeof(DebrisBodyWireV1) * debrisCap) +
           align64(sizeof(VehicleDestructionWireV1) * vehicleCap);
}

std::size_t segmentBytes(std::uint32_t debrisCap, std::uint32_t vehicleCap) noexcept {
    return align64(sizeof(DebrisHeaderV1)) + 3u * slotStride(debrisCap, vehicleCap);
}

DebrisBodyWireV1* slotBodies(DebrisSlotV1* s) noexcept {
    return reinterpret_cast<DebrisBodyWireV1*>(reinterpret_cast<unsigned char*>(s) + align64(sizeof(DebrisSlotV1)));
}

const DebrisBodyWireV1* slotBodies(const DebrisSlotV1* s) noexcept {
    return reinterpret_cast<const DebrisBodyWireV1*>(reinterpret_cast<const unsigned char*>(s) + align64(sizeof(DebrisSlotV1)));
}

VehicleDestructionWireV1* slotVehicles(DebrisSlotV1* s, std::uint32_t debrisCap) noexcept {
    return reinterpret_cast<VehicleDestructionWireV1*>(reinterpret_cast<unsigned char*>(s) + align64(sizeof(DebrisSlotV1)) +
                                                       align64(sizeof(DebrisBodyWireV1) * debrisCap));
}

const VehicleDestructionWireV1* slotVehicles(const DebrisSlotV1* s, std::uint32_t debrisCap) noexcept {
    return reinterpret_cast<const VehicleDestructionWireV1*>(reinterpret_cast<const unsigned char*>(s) + align64(sizeof(DebrisSlotV1)) +
                                                             align64(sizeof(DebrisBodyWireV1) * debrisCap));
}

void slerpQuat(const float a[4], const float b[4], float t, float out[4]) noexcept {
    float bx = b[0], by = b[1], bz = b[2], bw = b[3];
    float dot = a[0] * bx + a[1] * by + a[2] * bz + a[3] * bw;
    if (dot < 0.0f) {
        dot = -dot;
        bx = -bx; by = -by; bz = -bz; bw = -bw;
    }
    float w0 = 1.0f - t, w1 = t;
    if (dot < 0.9995f) {
        const float theta = std::acos(std::clamp(dot, -1.0f, 1.0f));
        const float s = std::sin(theta);
        if (s > 1e-6f) {
            w0 = std::sin((1.0f - t) * theta) / s;
            w1 = std::sin(t * theta) / s;
        }
    }
    out[0] = w0 * a[0] + w1 * bx;
    out[1] = w0 * a[1] + w1 * by;
    out[2] = w0 * a[2] + w1 * bz;
    out[3] = w0 * a[3] + w1 * bw;
    const float len = std::sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2] + out[3] * out[3]);
    if (len > 1e-6f) {
        const float inv = 1.0f / len;
        for (int k = 0; k < 4; ++k) out[k] *= inv;
    } else {
        out[0] = out[1] = out[2] = 0.0f;
        out[3] = 1.0f;
    }
}

struct DebrisFrameFileHeadV1 {
    double simTime = 0.0;
    std::uint32_t debrisCount = 0;
    std::uint32_t vehicleCount = 0;
};
static_assert(sizeof(DebrisFrameFileHeadV1) == 16, "debris frame head size");

} // namespace

DebrisSegmentV1::~DebrisSegmentV1() { close(); }

void DebrisSegmentV1::close() noexcept {
    if (owner_ && header_) header_->alive.store(0, std::memory_order_release);
    header_ = nullptr;
    owner_ = false;
    debrisCapacity_ = 0;
    vehicleCapacity_ = 0;
    memory_.close();
}

DebrisSlotV1* DebrisSegmentV1::slot(std::uint64_t sequence) const noexcept {
    return reinterpret_cast<DebrisSlotV1*>(static_cast<unsigned char*>(const_cast<void*>(memory_.data())) +
                                           align64(sizeof(DebrisHeaderV1)) +
                                           (sequence % 3u) * slotStride(debrisCapacity_, vehicleCapacity_));
}

bool DebrisSegmentV1::create(const std::string& world, std::uint32_t debrisCapacity, std::uint32_t vehicleCapacity,
                             std::uint64_t pid, std::uint32_t incarnation) {
    close();
    if (!debrisCapacity || debrisCapacity > kMaxDebrisV1 || !vehicleCapacity || vehicleCapacity > 65536u || !pid || !incarnation)
        return false;
    const std::size_t bytes = segmentBytes(debrisCapacity, vehicleCapacity);
    if (!memory_.create(segmentName(world), bytes)) return false;
    header_ = static_cast<DebrisHeaderV1*>(memory_.data());
    header_->magic = 0;
    new (header_) DebrisHeaderV1{};
    header_->debrisCapacity = debrisCapacity;
    header_->vehicleCapacity = vehicleCapacity;
    header_->publisherPid = pid;
    header_->incarnation = incarnation;
    debrisCapacity_ = debrisCapacity;
    vehicleCapacity_ = vehicleCapacity;
    pid_ = pid;
    incarnation_ = incarnation;
    owner_ = true;
    for (std::uint64_t i = 0; i < 3; ++i) new (slot(i)) DebrisSlotV1{};
    header_->alive.store(1, std::memory_order_release);
    header_->magic = kDebrisMagicV1;
    return true;
}

bool DebrisSegmentV1::open(const std::string& world, std::uint64_t pid, std::uint32_t incarnation) {
    close();
    if (!memory_.open(segmentName(world), sizeof(DebrisHeaderV1))) return false;
    auto* h = static_cast<DebrisHeaderV1*>(memory_.data());
    if (h->magic != kDebrisMagicV1 || h->version != kDebrisVersionV1 || !h->debrisCapacity || h->debrisCapacity > kMaxDebrisV1 ||
        !h->vehicleCapacity || h->vehicleCapacity > 65536u || h->publisherPid != pid || h->incarnation != incarnation ||
        !h->alive.load(std::memory_order_acquire)) {
        close();
        return false;
    }
    const std::uint32_t dc = h->debrisCapacity, vc = h->vehicleCapacity;
    memory_.close();
    if (!memory_.open(segmentName(world), segmentBytes(dc, vc))) return false;
    h = static_cast<DebrisHeaderV1*>(memory_.data());
    if (h->magic != kDebrisMagicV1 || h->version != kDebrisVersionV1 || h->debrisCapacity != dc || h->vehicleCapacity != vc ||
        h->publisherPid != pid || h->incarnation != incarnation || !h->alive.load(std::memory_order_acquire)) {
        close();
        return false;
    }
    header_ = h;
    debrisCapacity_ = dc;
    vehicleCapacity_ = vc;
    pid_ = pid;
    incarnation_ = incarnation;
    return true;
}

bool DebrisSegmentV1::publish(std::uint64_t sequence, const DebrisFrameV1& frame) noexcept {
    if (!owner_ || !header_ || !sequence) return false;
    auto* s = slot(sequence);
    s->seq.fetch_add(1, std::memory_order_acq_rel);
    s->sequence = 0;
    s->debrisCount = 0;
    s->vehicleCount = 0;
    s->simTime = frame.simTime;

    const std::uint32_t dc = static_cast<std::uint32_t>(std::min<std::size_t>(frame.bodies.size(), debrisCapacity_));
    const std::uint32_t vc = static_cast<std::uint32_t>(std::min<std::size_t>(frame.vehicles.size(), vehicleCapacity_));
    auto* dstBodies = slotBodies(s);
    std::uint32_t validBodies = 0;
    for (std::uint32_t i = 0; i < dc; ++i)
        if (validDebrisBodyV1(frame.bodies[i])) dstBodies[validBodies++] = frame.bodies[i];
    auto* dstVehicles = slotVehicles(s, debrisCapacity_);
    std::uint32_t validVehicles = 0;
    for (std::uint32_t i = 0; i < vc; ++i)
        if (validVehicleDestructionV1(frame.vehicles[i], vehicleCapacity_)) dstVehicles[validVehicles++] = frame.vehicles[i];

    s->debrisCount = validBodies;
    s->vehicleCount = validVehicles;
    s->sequence = sequence;
    s->seq.fetch_add(1, std::memory_order_release);
    return true;
}

bool DebrisSegmentV1::read(std::uint64_t sequence, DebrisFrameV1& out) const {
    out.bodies.clear();
    out.vehicles.clear();
    if (!header_ || !sequence || header_->magic != kDebrisMagicV1 || header_->publisherPid != pid_ ||
        header_->incarnation != incarnation_ || !header_->alive.load(std::memory_order_acquire))
        return false;
    for (unsigned attempt = 0; attempt < 8; ++attempt) {
        const auto* s = slot(sequence);
        const std::uint32_t before = s->seq.load(std::memory_order_acquire);
        if (before & 1u) continue;
        if (s->sequence != sequence || s->debrisCount > debrisCapacity_ || s->vehicleCount > vehicleCapacity_) return false;
        const std::uint32_t dc = s->debrisCount;
        const std::uint32_t vc = s->vehicleCount;
        const double simTime = s->simTime;
        std::vector<DebrisBodyWireV1> bodies(dc);
        std::vector<VehicleDestructionWireV1> vehicles(vc);
        if (dc) std::memcpy(bodies.data(), slotBodies(s), dc * sizeof(DebrisBodyWireV1));
        if (vc) std::memcpy(vehicles.data(), slotVehicles(s, debrisCapacity_), vc * sizeof(VehicleDestructionWireV1));
        std::atomic_thread_fence(std::memory_order_acquire);
        if (s->seq.load(std::memory_order_acquire) != before) continue;
        if (header_->publisherPid != pid_ || header_->incarnation != incarnation_ || !header_->alive.load(std::memory_order_acquire))
            return false;
        out.simTime = simTime;
        out.bodies = std::move(bodies);
        out.vehicles = std::move(vehicles);
        return true;
    }
    return false;
}

DebrisRecorderV1::~DebrisRecorderV1() { close(); }

bool DebrisRecorderV1::open(const std::filesystem::path& path) {
    close();
#ifdef _WIN32
    file_ = _wfopen(path.wstring().c_str(), L"wb");
#else
    file_ = std::fopen(path.string().c_str(), "wb");
#endif
    if (!file_) return false;
    const std::uint32_t version = kDebrisVersionV1;
    if (std::fwrite(kDebrisRecordMagicV1, 1, sizeof(kDebrisRecordMagicV1), file_) != sizeof(kDebrisRecordMagicV1) ||
        std::fwrite(&version, sizeof(version), 1, file_) != 1) {
        close();
        return false;
    }
    return true;
}

bool DebrisRecorderV1::write(const DebrisFrameV1& frame) {
    if (!file_) return false;
    const std::uint32_t dc = static_cast<std::uint32_t>(std::min<std::size_t>(frame.bodies.size(), kMaxDebrisV1));
    const std::uint32_t vc = static_cast<std::uint32_t>(std::min<std::size_t>(frame.vehicles.size(), kMaxVehicleDestructionV1));
    const DebrisFrameFileHeadV1 head{frame.simTime, dc, vc};
    if (std::fwrite(&head, sizeof(head), 1, file_) != 1) return false;
    if (dc && std::fwrite(frame.bodies.data(), sizeof(DebrisBodyWireV1), dc, file_) != dc) return false;
    if (vc && std::fwrite(frame.vehicles.data(), sizeof(VehicleDestructionWireV1), vc, file_) != vc) return false;
    return true;
}

void DebrisRecorderV1::close() {
    if (!file_) return;
    std::fflush(file_);
    std::fclose(file_);
    file_ = nullptr;
}

bool DebrisReplayV1::load(const std::filesystem::path& path, std::string* error) {
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
    char magic[sizeof(kDebrisRecordMagicV1)] = {};
    std::uint32_t version = 0;
    if (std::fread(magic, 1, sizeof(magic), f) != sizeof(magic) ||
        std::memcmp(magic, kDebrisRecordMagicV1, sizeof(magic)) != 0 ||
        std::fread(&version, sizeof(version), 1, f) != 1 || version != kDebrisVersionV1) {
        std::fclose(f);
        if (error) *error = "invalid debris sidecar header: " + path.string();
        return false;
    }
    for (;;) {
        DebrisFrameFileHeadV1 head;
        if (std::fread(&head, sizeof(head), 1, f) != 1) break;
        if (!std::isfinite(head.simTime) || head.debrisCount > kMaxDebrisV1 || head.vehicleCount > kMaxVehicleDestructionV1) break;
        DebrisFrameV1 frame;
        frame.simTime = head.simTime;
        frame.bodies.resize(head.debrisCount);
        frame.vehicles.resize(head.vehicleCount);
        if (head.debrisCount && std::fread(frame.bodies.data(), sizeof(DebrisBodyWireV1), head.debrisCount, f) != head.debrisCount) break;
        if (head.vehicleCount &&
            std::fread(frame.vehicles.data(), sizeof(VehicleDestructionWireV1), head.vehicleCount, f) != head.vehicleCount)
            break;
        frames_.push_back(std::move(frame));
    }
    std::fclose(f);
    return !frames_.empty();
}

bool DebrisReplayV1::atTime(double simTimeS, DebrisFrameV1& out, bool interpolate) const {
    out.bodies.clear();
    out.vehicles.clear();
    out.simTime = simTimeS;
    if (frames_.empty() || !std::isfinite(simTimeS)) return false;
    if (simTimeS <= frames_.front().simTime) {
        if (frames_.front().simTime - simTimeS > 0.25) return false;
        out = frames_.front();
        return true;
    }
    if (simTimeS >= frames_.back().simTime) {
        out = frames_.back();
        return true;
    }
    const auto it = std::upper_bound(frames_.begin(), frames_.end(), simTimeS,
                                     [](double t, const DebrisFrameV1& f) { return t < f.simTime; });
    const DebrisFrameV1& b = *it;
    const DebrisFrameV1& a = *(it - 1);
    const double span = b.simTime - a.simTime;
    if (!interpolate || !(span > 1e-9)) {
        out = a;
        return true;
    }
    const double alpha = std::clamp((simTimeS - a.simTime) / span, 0.0, 1.0);
    const float fa = static_cast<float>(alpha);
    out.simTime = simTimeS;
    out.vehicles = alpha < 0.5 ? a.vehicles : b.vehicles;

    std::unordered_map<std::uint32_t, const DebrisBodyWireV1*> nextById;
    nextById.reserve(b.bodies.size());
    for (const auto& body : b.bodies) nextById[body.id] = &body;

    out.bodies.reserve(a.bodies.size());
    for (const auto& ba : a.bodies) {
        const auto found = nextById.find(ba.id);
        if (found == nextById.end()) {
            // Extrapolate smoothly until the chunk retires at the next boundary.
            DebrisBodyWireV1 e = ba;
            const double dt = simTimeS - a.simTime;
            for (int k = 0; k < 3; ++k) e.positionEcef[k] += ba.velocityEcef[k] * dt;
            e.ageS += static_cast<float>(dt);
            out.bodies.push_back(e);
            continue;
        }
        const auto& bb = *found->second;
        DebrisBodyWireV1 m = ba;
        for (int k = 0; k < 3; ++k) {
            m.positionEcef[k] = ba.positionEcef[k] + alpha * (bb.positionEcef[k] - ba.positionEcef[k]);
            m.velocityEcef[k] = ba.velocityEcef[k] + alpha * (bb.velocityEcef[k] - ba.velocityEcef[k]);
        }
        m.temperatureK = ba.temperatureK + fa * (bb.temperatureK - ba.temperatureK);
        m.smokeIntensity = ba.smokeIntensity + fa * (bb.smokeIntensity - ba.smokeIntensity);
        m.ageS = ba.ageS + fa * (bb.ageS - ba.ageS);
        slerpQuat(ba.quatBodyToEcef, bb.quatBodyToEcef, fa, m.quatBodyToEcef);
        if (alpha >= 0.5) m.flags = bb.flags;
        out.bodies.push_back(m);
    }
    return true;
}

} // namespace fsim::ipc
