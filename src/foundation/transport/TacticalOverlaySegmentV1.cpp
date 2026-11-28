#include "foundation/transport/TacticalOverlaySegmentV1.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <new>

namespace fsim::ipc {

namespace {

std::string segmentName(const std::string& world) { return "fsim.tact.v1." + world; }

std::size_t align64(std::size_t n) noexcept { return (n + 63u) & ~static_cast<std::size_t>(63u); }

struct SlotOffsets {
    std::size_t radars = 0;
    std::size_t jammers = 0;
    std::size_t esm = 0;
    std::size_t links = 0;
    std::size_t weapons = 0;
    std::size_t swaths = 0;
    std::size_t props = 0;
    std::size_t stride = 0;
};

SlotOffsets offsetsFor(std::uint32_t rCap, std::uint32_t jCap, std::uint32_t eCap, std::uint32_t lCap,
                       std::uint32_t wCap, std::uint32_t sCap, std::uint32_t pCap) noexcept {
    SlotOffsets o;
    o.radars = align64(sizeof(TacticalOverlaySlotV1));
    o.jammers = o.radars + align64(sizeof(TacticalRadarVolumeWireV1) * rCap);
    o.esm = o.jammers + align64(sizeof(TacticalJammerLobeWireV1) * jCap);
    o.links = o.esm + align64(sizeof(TacticalEsmRayWireV1) * eCap);
    o.weapons = o.links + align64(sizeof(TacticalLinkPulseWireV1) * lCap);
    o.swaths = o.weapons + align64(sizeof(TacticalWeaponCueWireV1) * wCap);
    o.props = o.swaths + align64(sizeof(TacticalSwathWireV1) * sCap);
    o.stride = o.props + align64(sizeof(TacticalPropulsionExtWireV1) * pCap);
    return o;
}

std::size_t segmentBytes(std::uint32_t rCap, std::uint32_t jCap, std::uint32_t eCap, std::uint32_t lCap,
                         std::uint32_t wCap, std::uint32_t sCap, std::uint32_t pCap) noexcept {
    return align64(sizeof(TacticalOverlayHeaderV1)) + 3u * offsetsFor(rCap, jCap, eCap, lCap, wCap, sCap, pCap).stride;
}

struct TacticalFileHeadV1 {
    double simTime = 0.0;
    std::uint32_t radarCount = 0;
    std::uint32_t jammerCount = 0;
    std::uint32_t esmCount = 0;
    std::uint32_t linkCount = 0;
    std::uint32_t weaponCount = 0;
    std::uint32_t swathCount = 0;
    std::uint32_t propulsionCount = 0;
    std::uint32_t reserved = 0;
};
static_assert(sizeof(TacticalFileHeadV1) == 40, "tactical file head size");

} // namespace

TacticalOverlaySegmentV1::~TacticalOverlaySegmentV1() { close(); }

void TacticalOverlaySegmentV1::close() noexcept {
    if (owner_ && header_) header_->alive.store(0, std::memory_order_release);
    header_ = nullptr;
    owner_ = false;
    radarCap_ = jammerCap_ = esmCap_ = linkCap_ = weaponCap_ = swathCap_ = propCap_ = 0;
    memory_.close();
}

TacticalOverlaySlotV1* TacticalOverlaySegmentV1::slot(std::uint64_t sequence) const noexcept {
    const auto o = offsetsFor(radarCap_, jammerCap_, esmCap_, linkCap_, weaponCap_, swathCap_, propCap_);
    return reinterpret_cast<TacticalOverlaySlotV1*>(
        static_cast<unsigned char*>(const_cast<void*>(memory_.data())) +
        align64(sizeof(TacticalOverlayHeaderV1)) + (sequence % 3u) * o.stride);
}

bool TacticalOverlaySegmentV1::create(const std::string& world, std::uint64_t pid, std::uint32_t incarnation) {
    close();
    if (!pid || !incarnation) return false;
    const std::size_t bytes = segmentBytes(kMaxTacticalRadarsV1, kMaxTacticalJammersV1, kMaxTacticalEsmRaysV1,
                                           kMaxTacticalLinksV1, kMaxTacticalWeaponsV1, kMaxTacticalSwathsV1,
                                           kMaxTacticalPropulsionV1);
    if (!memory_.create(segmentName(world), bytes)) return false;
    header_ = static_cast<TacticalOverlayHeaderV1*>(memory_.data());
    header_->magic = 0;
    new (header_) TacticalOverlayHeaderV1{};
    header_->radarCapacity = radarCap_ = kMaxTacticalRadarsV1;
    header_->jammerCapacity = jammerCap_ = kMaxTacticalJammersV1;
    header_->esmCapacity = esmCap_ = kMaxTacticalEsmRaysV1;
    header_->linkCapacity = linkCap_ = kMaxTacticalLinksV1;
    header_->weaponCapacity = weaponCap_ = kMaxTacticalWeaponsV1;
    header_->swathCapacity = swathCap_ = kMaxTacticalSwathsV1;
    header_->propulsionCapacity = propCap_ = kMaxTacticalPropulsionV1;
    header_->publisherPid = pid_ = pid;
    header_->incarnation = incarnation_ = incarnation;
    owner_ = true;
    for (std::uint64_t i = 0; i < 3; ++i) new (slot(i)) TacticalOverlaySlotV1{};
    header_->alive.store(1, std::memory_order_release);
    header_->magic = kTacticalMagicV1;
    return true;
}

bool TacticalOverlaySegmentV1::open(const std::string& world, std::uint64_t pid, std::uint32_t incarnation) {
    close();
    if (!memory_.open(segmentName(world), sizeof(TacticalOverlayHeaderV1))) return false;
    auto* h = static_cast<TacticalOverlayHeaderV1*>(memory_.data());
    if (h->magic != kTacticalMagicV1 || h->version != kTacticalVersionV1 || h->publisherPid != pid ||
        h->incarnation != incarnation || h->radarCapacity > kMaxTacticalRadarsV1 ||
        h->jammerCapacity > kMaxTacticalJammersV1 || h->esmCapacity > kMaxTacticalEsmRaysV1 ||
        h->linkCapacity > kMaxTacticalLinksV1 || h->weaponCapacity > kMaxTacticalWeaponsV1 ||
        h->swathCapacity > kMaxTacticalSwathsV1 || h->propulsionCapacity > kMaxTacticalPropulsionV1) {
        close();
        return false;
    }
    const std::uint32_t rCap = h->radarCapacity, jCap = h->jammerCapacity, eCap = h->esmCapacity;
    const std::uint32_t lCap = h->linkCapacity, wCap = h->weaponCapacity, sCap = h->swathCapacity, pCap = h->propulsionCapacity;
    const std::size_t bytes = segmentBytes(rCap, jCap, eCap, lCap, wCap, sCap, pCap);
    memory_.close();
    if (!memory_.open(segmentName(world), bytes)) return false;
    header_ = static_cast<TacticalOverlayHeaderV1*>(memory_.data());
    if (header_->magic != kTacticalMagicV1 || header_->version != kTacticalVersionV1 ||
        header_->publisherPid != pid || header_->incarnation != incarnation) {
        close();
        return false;
    }
    radarCap_ = rCap;
    jammerCap_ = jCap;
    esmCap_ = eCap;
    linkCap_ = lCap;
    weaponCap_ = wCap;
    swathCap_ = sCap;
    propCap_ = pCap;
    pid_ = pid;
    incarnation_ = incarnation;
    owner_ = false;
    return true;
}

bool TacticalOverlaySegmentV1::publish(std::uint64_t sequence, const TacticalOverlayFrameV1& frame) noexcept {
    if (!owner_ || !header_) return false;
    auto* s = slot(sequence);
    const auto o = offsetsFor(radarCap_, jammerCap_, esmCap_, linkCap_, weaponCap_, swathCap_, propCap_);
    auto* base = reinterpret_cast<unsigned char*>(s);
    const std::uint32_t seq0 = s->seq.load(std::memory_order_relaxed);
    s->seq.store(seq0 + 1u, std::memory_order_release);
    std::atomic_thread_fence(std::memory_order_acq_rel);

    const auto copyArray = [&](auto* dst, const auto& src, std::uint32_t cap, auto validator) -> std::uint32_t {
        std::uint32_t n = 0;
        for (const auto& item : src) {
            if (n >= cap) break;
            if (validator(item)) dst[n++] = item;
        }
        return n;
    };
    s->radarCount = copyArray(reinterpret_cast<TacticalRadarVolumeWireV1*>(base + o.radars), frame.radars, radarCap_, validTacticalRadarV1);
    s->jammerCount = copyArray(reinterpret_cast<TacticalJammerLobeWireV1*>(base + o.jammers), frame.jammers, jammerCap_, validTacticalJammerV1);
    s->esmCount = copyArray(reinterpret_cast<TacticalEsmRayWireV1*>(base + o.esm), frame.esmRays, esmCap_, validTacticalEsmRayV1);
    s->linkCount = copyArray(reinterpret_cast<TacticalLinkPulseWireV1*>(base + o.links), frame.links, linkCap_, validTacticalLinkPulseV1);
    s->weaponCount = copyArray(reinterpret_cast<TacticalWeaponCueWireV1*>(base + o.weapons), frame.weapons, weaponCap_, validTacticalWeaponCueV1);
    s->swathCount = copyArray(reinterpret_cast<TacticalSwathWireV1*>(base + o.swaths), frame.swaths, swathCap_, validTacticalSwathV1);
    s->propulsionCount = copyArray(reinterpret_cast<TacticalPropulsionExtWireV1*>(base + o.props), frame.propulsion, propCap_, validTacticalPropulsionExtV1);
    s->sequence = sequence;
    s->simTime = std::isfinite(frame.simTime) ? frame.simTime : 0.0;

    std::atomic_thread_fence(std::memory_order_release);
    s->seq.store(seq0 + 2u, std::memory_order_release);
    return true;
}

bool TacticalOverlaySegmentV1::read(std::uint64_t sequence, TacticalOverlayFrameV1& out) const {
    if (!header_) return false;
    if (header_->magic != kTacticalMagicV1 || header_->version != kTacticalVersionV1 ||
        header_->publisherPid != pid_ || header_->incarnation != incarnation_)
        return false;
    const auto* s = slot(sequence);
    const auto o = offsetsFor(radarCap_, jammerCap_, esmCap_, linkCap_, weaponCap_, swathCap_, propCap_);
    const auto* base = reinterpret_cast<const unsigned char*>(s);
    for (int attempt = 0; attempt < 8; ++attempt) {
        const std::uint32_t seq0 = s->seq.load(std::memory_order_acquire);
        if (seq0 & 1u) continue;
        if (s->sequence != sequence) return false;
        const std::uint32_t rN = std::min(s->radarCount, radarCap_);
        const std::uint32_t jN = std::min(s->jammerCount, jammerCap_);
        const std::uint32_t eN = std::min(s->esmCount, esmCap_);
        const std::uint32_t lN = std::min(s->linkCount, linkCap_);
        const std::uint32_t wN = std::min(s->weaponCount, weaponCap_);
        const std::uint32_t sN = std::min(s->swathCount, swathCap_);
        const std::uint32_t pN = std::min(s->propulsionCount, propCap_);
        TacticalOverlayFrameV1 tmp;
        tmp.simTime = s->simTime;
        const auto* rPtr = reinterpret_cast<const TacticalRadarVolumeWireV1*>(base + o.radars);
        const auto* jPtr = reinterpret_cast<const TacticalJammerLobeWireV1*>(base + o.jammers);
        const auto* ePtr = reinterpret_cast<const TacticalEsmRayWireV1*>(base + o.esm);
        const auto* lPtr = reinterpret_cast<const TacticalLinkPulseWireV1*>(base + o.links);
        const auto* wPtr = reinterpret_cast<const TacticalWeaponCueWireV1*>(base + o.weapons);
        const auto* sPtr = reinterpret_cast<const TacticalSwathWireV1*>(base + o.swaths);
        const auto* pPtr = reinterpret_cast<const TacticalPropulsionExtWireV1*>(base + o.props);
        tmp.radars.assign(rPtr, rPtr + rN);
        tmp.jammers.assign(jPtr, jPtr + jN);
        tmp.esmRays.assign(ePtr, ePtr + eN);
        tmp.links.assign(lPtr, lPtr + lN);
        tmp.weapons.assign(wPtr, wPtr + wN);
        tmp.swaths.assign(sPtr, sPtr + sN);
        tmp.propulsion.assign(pPtr, pPtr + pN);
        std::atomic_thread_fence(std::memory_order_acquire);
        if (s->seq.load(std::memory_order_acquire) != seq0) continue;
        out = std::move(tmp);
        return true;
    }
    return false;
}

// --- Companion .tact recorder & reader ---------------------------------------

TacticalOverlayRecorderV1::~TacticalOverlayRecorderV1() { close(); }

bool TacticalOverlayRecorderV1::open(const std::filesystem::path& path) {
    close();
    file_ = std::fopen(path.string().c_str(), "wb");
    if (!file_) return false;
    const std::uint32_t version = kTacticalVersionV1;
    if (std::fwrite(kTacticalRecordMagicV1, 1, sizeof kTacticalRecordMagicV1, file_) != sizeof kTacticalRecordMagicV1 ||
        std::fwrite(&version, sizeof version, 1, file_) != 1) {
        close();
        return false;
    }
    return true;
}

bool TacticalOverlayRecorderV1::write(const TacticalOverlayFrameV1& frame) {
    if (!file_) return false;
    TacticalFileHeadV1 head{};
    head.simTime = std::isfinite(frame.simTime) ? frame.simTime : 0.0;
    head.radarCount = static_cast<std::uint32_t>(std::min<std::size_t>(frame.radars.size(), kMaxTacticalRadarsV1));
    head.jammerCount = static_cast<std::uint32_t>(std::min<std::size_t>(frame.jammers.size(), kMaxTacticalJammersV1));
    head.esmCount = static_cast<std::uint32_t>(std::min<std::size_t>(frame.esmRays.size(), kMaxTacticalEsmRaysV1));
    head.linkCount = static_cast<std::uint32_t>(std::min<std::size_t>(frame.links.size(), kMaxTacticalLinksV1));
    head.weaponCount = static_cast<std::uint32_t>(std::min<std::size_t>(frame.weapons.size(), kMaxTacticalWeaponsV1));
    head.swathCount = static_cast<std::uint32_t>(std::min<std::size_t>(frame.swaths.size(), kMaxTacticalSwathsV1));
    head.propulsionCount = static_cast<std::uint32_t>(std::min<std::size_t>(frame.propulsion.size(), kMaxTacticalPropulsionV1));
    const char tag = 'T';
    if (std::fwrite(&tag, 1, 1, file_) != 1 || std::fwrite(&head, sizeof head, 1, file_) != 1) return false;
    auto writeVec = [&](const auto& vec, std::uint32_t n) -> bool {
        return n == 0 || std::fwrite(vec.data(), sizeof(vec[0]), n, file_) == n;
    };
    const bool ok = writeVec(frame.radars, head.radarCount) && writeVec(frame.jammers, head.jammerCount) &&
                    writeVec(frame.esmRays, head.esmCount) && writeVec(frame.links, head.linkCount) &&
                    writeVec(frame.weapons, head.weaponCount) && writeVec(frame.swaths, head.swathCount) &&
                    writeVec(frame.propulsion, head.propulsionCount);
    if (!ok) return false;
    if (!frame.radarTracks.empty()) {
        const std::uint32_t kCount = static_cast<std::uint32_t>(std::min<std::size_t>(frame.radarTracks.size(), kMaxTacticalRadarTracksV1));
        const char kTag = 'K';
        if (std::fwrite(&kTag, 1, 1, file_) != 1 || std::fwrite(&kCount, sizeof kCount, 1, file_) != 1 ||
            !writeVec(frame.radarTracks, kCount)) {
            return false;
        }
    }
    return true;
}

void TacticalOverlayRecorderV1::close() {
    if (file_) {
        std::fclose(file_);
        file_ = nullptr;
    }
}

bool TacticalOverlayReplayV1::load(const std::filesystem::path& path, std::string* error) {
    frames_.clear();
    std::FILE* f = std::fopen(path.string().c_str(), "rb");
    if (!f) {
        if (error) *error = "unable to open tactical replay sidecar";
        return false;
    }
    char magic[6] = {};
    std::uint32_t version = 0;
    if (std::fread(magic, 1, sizeof magic, f) != sizeof magic ||
        std::memcmp(magic, kTacticalRecordMagicV1, sizeof magic) != 0 ||
        std::fread(&version, sizeof version, 1, f) != 1 || version != kTacticalVersionV1) {
        std::fclose(f);
        if (error) *error = "invalid tactical replay header";
        return false;
    }
    for (;;) {
        char tag = 0;
        if (std::fread(&tag, 1, 1, f) != 1) break;
        if (tag == 'K') {
            std::uint32_t kCount = 0;
            if (frames_.empty() || std::fread(&kCount, sizeof kCount, 1, f) != 1 || kCount > kMaxTacticalRadarTracksV1) break;
            auto& dst = frames_.back().radarTracks;
            dst.resize(kCount);
            if (kCount > 0 && std::fread(dst.data(), sizeof(dst[0]), kCount, f) != kCount) break;
            continue;
        }
        if (tag != 'T') break;
        TacticalFileHeadV1 head{};
        if (std::fread(&head, sizeof head, 1, f) != 1) break;
        if (head.radarCount > kMaxTacticalRadarsV1 || head.jammerCount > kMaxTacticalJammersV1 ||
            head.esmCount > kMaxTacticalEsmRaysV1 || head.linkCount > kMaxTacticalLinksV1 ||
            head.weaponCount > kMaxTacticalWeaponsV1 || head.swathCount > kMaxTacticalSwathsV1 ||
            head.propulsionCount > kMaxTacticalPropulsionV1) {
            break;
        }
        TacticalOverlayFrameV1 frame;
        frame.simTime = head.simTime;
        auto readVec = [&](auto& vec, std::uint32_t n) -> bool {
            vec.resize(n);
            return n == 0 || std::fread(vec.data(), sizeof(vec[0]), n, f) == n;
        };
        if (!readVec(frame.radars, head.radarCount) || !readVec(frame.jammers, head.jammerCount) ||
            !readVec(frame.esmRays, head.esmCount) || !readVec(frame.links, head.linkCount) ||
            !readVec(frame.weapons, head.weaponCount) || !readVec(frame.swaths, head.swathCount) ||
            !readVec(frame.propulsion, head.propulsionCount))
            break;
        frames_.push_back(std::move(frame));
    }
    std::fclose(f);
    return !frames_.empty();
}

bool TacticalOverlayReplayV1::atTime(double simTimeS, TacticalOverlayFrameV1& out) const {
    if (frames_.empty() || !std::isfinite(simTimeS)) return false;
    auto it = std::upper_bound(frames_.begin(), frames_.end(), simTimeS,
                               [](double t, const TacticalOverlayFrameV1& f) { return t < f.simTime; });
    if (it == frames_.begin()) out = frames_.front();
    else out = *std::prev(it);
    return true;
}

} // namespace fsim::ipc
