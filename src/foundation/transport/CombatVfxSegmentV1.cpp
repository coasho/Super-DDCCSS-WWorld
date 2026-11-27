#include "foundation/transport/CombatVfxSegmentV1.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <new>
#include <unordered_map>
#include <unordered_set>

namespace fsim::ipc {

namespace {

std::string segmentName(const std::string& world) { return "fsim.vfx.v1." + world; }

std::size_t align64(std::size_t n) noexcept { return (n + 63u) & ~static_cast<std::size_t>(63u); }

std::size_t slotStride(std::uint32_t projCap, std::uint32_t decoyCap, std::uint32_t evtCap) noexcept {
    return align64(sizeof(CombatVfxSlotV1)) + align64(sizeof(ProjectileWireV1) * projCap) +
           align64(sizeof(DecoyWireV1) * decoyCap) + align64(sizeof(CombatVfxEventWireV1) * evtCap);
}

std::size_t segmentBytes(std::uint32_t projCap, std::uint32_t decoyCap, std::uint32_t evtCap) noexcept {
    return align64(sizeof(CombatVfxHeaderV1)) + 3u * slotStride(projCap, decoyCap, evtCap);
}

ProjectileWireV1* slotProjectiles(CombatVfxSlotV1* s) noexcept {
    return reinterpret_cast<ProjectileWireV1*>(reinterpret_cast<unsigned char*>(s) + align64(sizeof(CombatVfxSlotV1)));
}

const ProjectileWireV1* slotProjectiles(const CombatVfxSlotV1* s) noexcept {
    return reinterpret_cast<const ProjectileWireV1*>(reinterpret_cast<const unsigned char*>(s) + align64(sizeof(CombatVfxSlotV1)));
}

DecoyWireV1* slotDecoys(CombatVfxSlotV1* s, std::uint32_t projCap) noexcept {
    return reinterpret_cast<DecoyWireV1*>(reinterpret_cast<unsigned char*>(s) + align64(sizeof(CombatVfxSlotV1)) +
                                          align64(sizeof(ProjectileWireV1) * projCap));
}

const DecoyWireV1* slotDecoys(const CombatVfxSlotV1* s, std::uint32_t projCap) noexcept {
    return reinterpret_cast<const DecoyWireV1*>(reinterpret_cast<const unsigned char*>(s) + align64(sizeof(CombatVfxSlotV1)) +
                                                align64(sizeof(ProjectileWireV1) * projCap));
}

CombatVfxEventWireV1* slotEvents(CombatVfxSlotV1* s, std::uint32_t projCap, std::uint32_t decoyCap) noexcept {
    return reinterpret_cast<CombatVfxEventWireV1*>(reinterpret_cast<unsigned char*>(s) + align64(sizeof(CombatVfxSlotV1)) +
                                                   align64(sizeof(ProjectileWireV1) * projCap) +
                                                   align64(sizeof(DecoyWireV1) * decoyCap));
}

const CombatVfxEventWireV1* slotEvents(const CombatVfxSlotV1* s, std::uint32_t projCap, std::uint32_t decoyCap) noexcept {
    return reinterpret_cast<const CombatVfxEventWireV1*>(reinterpret_cast<const unsigned char*>(s) + align64(sizeof(CombatVfxSlotV1)) +
                                                         align64(sizeof(ProjectileWireV1) * projCap) +
                                                         align64(sizeof(DecoyWireV1) * decoyCap));
}

struct CombatVfxFrameFileHeadV1 {
    double simTime = 0.0;
    std::uint32_t projectileCount = 0;
    std::uint32_t decoyCount = 0;
    std::uint32_t eventCount = 0;
    std::uint32_t reserved = 0;
};
static_assert(sizeof(CombatVfxFrameFileHeadV1) == 24, "combat VFX frame head size");

} // namespace

CombatVfxSegmentV1::~CombatVfxSegmentV1() { close(); }

void CombatVfxSegmentV1::close() noexcept {
    if (owner_ && header_) header_->alive.store(0, std::memory_order_release);
    header_ = nullptr;
    owner_ = false;
    projectileCapacity_ = 0;
    decoyCapacity_ = 0;
    eventCapacity_ = 0;
    memory_.close();
}

CombatVfxSlotV1* CombatVfxSegmentV1::slot(std::uint64_t sequence) const noexcept {
    return reinterpret_cast<CombatVfxSlotV1*>(static_cast<unsigned char*>(const_cast<void*>(memory_.data())) +
                                              align64(sizeof(CombatVfxHeaderV1)) +
                                              (sequence % 3u) * slotStride(projectileCapacity_, decoyCapacity_, eventCapacity_));
}

bool CombatVfxSegmentV1::create(const std::string& world, std::uint32_t projectileCapacity, std::uint32_t decoyCapacity,
                                std::uint32_t eventCapacity, std::uint64_t pid, std::uint32_t incarnation) {
    close();
    if (!projectileCapacity || projectileCapacity > kMaxProjectilesV1 || !decoyCapacity || decoyCapacity > kMaxDecoysV1 ||
        !eventCapacity || eventCapacity > kMaxCombatVfxEventsV1 || !pid || !incarnation)
        return false;
    const std::size_t bytes = segmentBytes(projectileCapacity, decoyCapacity, eventCapacity);
    if (!memory_.create(segmentName(world), bytes)) return false;
    header_ = static_cast<CombatVfxHeaderV1*>(memory_.data());
    header_->magic = 0;
    new (header_) CombatVfxHeaderV1{};
    header_->projectileCapacity = projectileCapacity;
    header_->decoyCapacity = decoyCapacity;
    header_->eventCapacity = eventCapacity;
    header_->publisherPid = pid;
    header_->incarnation = incarnation;
    projectileCapacity_ = projectileCapacity;
    decoyCapacity_ = decoyCapacity;
    eventCapacity_ = eventCapacity;
    pid_ = pid;
    incarnation_ = incarnation;
    owner_ = true;
    for (std::uint64_t i = 0; i < 3; ++i) new (slot(i)) CombatVfxSlotV1{};
    header_->alive.store(1, std::memory_order_release);
    header_->magic = kCombatVfxMagicV1;
    return true;
}

bool CombatVfxSegmentV1::open(const std::string& world, std::uint64_t pid, std::uint32_t incarnation) {
    close();
    if (!memory_.open(segmentName(world), sizeof(CombatVfxHeaderV1))) return false;
    auto* h = static_cast<CombatVfxHeaderV1*>(memory_.data());
    if (h->magic != kCombatVfxMagicV1 || h->version != kCombatVfxVersionV1 || !h->projectileCapacity ||
        h->projectileCapacity > kMaxProjectilesV1 || !h->decoyCapacity || h->decoyCapacity > kMaxDecoysV1 ||
        !h->eventCapacity || h->eventCapacity > kMaxCombatVfxEventsV1 || h->publisherPid != pid ||
        h->incarnation != incarnation || !h->alive.load(std::memory_order_acquire)) {
        close();
        return false;
    }
    const std::uint32_t pc = h->projectileCapacity, dc = h->decoyCapacity, ec = h->eventCapacity;
    memory_.close();
    if (!memory_.open(segmentName(world), segmentBytes(pc, dc, ec))) return false;
    h = static_cast<CombatVfxHeaderV1*>(memory_.data());
    if (h->magic != kCombatVfxMagicV1 || h->version != kCombatVfxVersionV1 || h->projectileCapacity != pc ||
        h->decoyCapacity != dc || h->eventCapacity != ec || h->publisherPid != pid ||
        h->incarnation != incarnation || !h->alive.load(std::memory_order_acquire)) {
        close();
        return false;
    }
    header_ = h;
    projectileCapacity_ = pc;
    decoyCapacity_ = dc;
    eventCapacity_ = ec;
    pid_ = pid;
    incarnation_ = incarnation;
    owner_ = false;
    return true;
}

bool CombatVfxSegmentV1::publish(std::uint64_t sequence, const CombatVfxFrameV1& frame) noexcept {
    if (!owner_ || !header_ || !sequence) return false;
    CombatVfxSlotV1* s = slot(sequence);
    s->seq.fetch_add(1, std::memory_order_acq_rel);
    s->sequence = sequence;
    s->simTime = std::isfinite(frame.simTime) ? frame.simTime : 0.0;

    std::uint32_t pc = 0;
    ProjectileWireV1* dstP = slotProjectiles(s);
    for (const auto& p : frame.projectiles) {
        if (pc >= projectileCapacity_) break;
        if (!validProjectileWireV1(p)) continue;
        dstP[pc++] = p;
    }
    s->projectileCount = pc;

    std::uint32_t dc = 0;
    DecoyWireV1* dstD = slotDecoys(s, projectileCapacity_);
    for (const auto& d : frame.decoys) {
        if (dc >= decoyCapacity_) break;
        if (!validDecoyWireV1(d)) continue;
        dstD[dc++] = d;
    }
    s->decoyCount = dc;

    std::uint32_t ec = 0;
    CombatVfxEventWireV1* dstE = slotEvents(s, projectileCapacity_, decoyCapacity_);
    for (const auto& e : frame.events) {
        if (ec >= eventCapacity_) break;
        if (!validCombatVfxEventV1(e)) continue;
        dstE[ec++] = e;
    }
    s->eventCount = ec;

    s->seq.fetch_add(1, std::memory_order_acq_rel);
    return true;
}

bool CombatVfxSegmentV1::read(std::uint64_t sequence, CombatVfxFrameV1& out) const {
    out.projectiles.clear();
    out.decoys.clear();
    out.events.clear();
    if (!header_ || !sequence || header_->magic != kCombatVfxMagicV1 || header_->publisherPid != pid_ ||
        header_->incarnation != incarnation_ || !header_->alive.load(std::memory_order_acquire))
        return false;
    const CombatVfxSlotV1* s = slot(sequence);
    for (int attempt = 0; attempt < 8; ++attempt) {
        const std::uint32_t s0 = s->seq.load(std::memory_order_acquire);
        if (s0 & 1u) continue;
        const std::uint64_t seq = s->sequence;
        const double simTime = s->simTime;
        const std::uint32_t pc = s->projectileCount;
        const std::uint32_t dc = s->decoyCount;
        const std::uint32_t ec = s->eventCount;
        if (seq != sequence || pc > projectileCapacity_ || dc > decoyCapacity_ || ec > eventCapacity_) {
            std::atomic_thread_fence(std::memory_order_acquire);
            if (s->seq.load(std::memory_order_acquire) == s0) return false;
            continue;
        }
        out.projectiles.resize(pc);
        if (pc) std::memcpy(out.projectiles.data(), slotProjectiles(s), pc * sizeof(ProjectileWireV1));
        out.decoys.resize(dc);
        if (dc) std::memcpy(out.decoys.data(), slotDecoys(s, projectileCapacity_), dc * sizeof(DecoyWireV1));
        out.events.resize(ec);
        if (ec) std::memcpy(out.events.data(), slotEvents(s, projectileCapacity_, decoyCapacity_), ec * sizeof(CombatVfxEventWireV1));
        std::atomic_thread_fence(std::memory_order_acquire);
        if (s->seq.load(std::memory_order_acquire) != s0) continue;
        out.simTime = simTime;
        out.projectiles.erase(std::remove_if(out.projectiles.begin(), out.projectiles.end(),
                                             [](const ProjectileWireV1& p) { return !validProjectileWireV1(p); }),
                              out.projectiles.end());
        out.decoys.erase(std::remove_if(out.decoys.begin(), out.decoys.end(),
                                        [](const DecoyWireV1& d) { return !validDecoyWireV1(d); }),
                         out.decoys.end());
        out.events.erase(std::remove_if(out.events.begin(), out.events.end(),
                                        [](const CombatVfxEventWireV1& e) { return !validCombatVfxEventV1(e); }),
                         out.events.end());
        return true;
    }
    out.projectiles.clear();
    out.decoys.clear();
    out.events.clear();
    return false;
}

// --- Companion .vfx sidecar recorder / replay ---

CombatVfxRecorderV1::~CombatVfxRecorderV1() { close(); }

bool CombatVfxRecorderV1::open(const std::filesystem::path& path) {
    close();
    if (path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(path.parent_path(), ec);
    }
    file_ = std::fopen(path.string().c_str(), "wb");
    if (!file_) return false;
    if (std::fwrite(kCombatVfxRecordMagicV1, 1, sizeof kCombatVfxRecordMagicV1, file_) != sizeof kCombatVfxRecordMagicV1) {
        close();
        return false;
    }
    const std::uint16_t version = static_cast<std::uint16_t>(kCombatVfxVersionV1);
    if (std::fwrite(&version, sizeof version, 1, file_) != 1) {
        close();
        return false;
    }
    return true;
}

bool CombatVfxRecorderV1::write(const CombatVfxFrameV1& frame) {
    if (!file_) return false;
    std::vector<ProjectileWireV1> validP;
    validP.reserve(std::min<std::size_t>(frame.projectiles.size(), kMaxProjectilesV1));
    for (const auto& p : frame.projectiles) {
        if (validP.size() >= kMaxProjectilesV1) break;
        if (validProjectileWireV1(p)) validP.push_back(p);
    }
    std::vector<DecoyWireV1> validD;
    validD.reserve(std::min<std::size_t>(frame.decoys.size(), kMaxDecoysV1));
    for (const auto& d : frame.decoys) {
        if (validD.size() >= kMaxDecoysV1) break;
        if (validDecoyWireV1(d)) validD.push_back(d);
    }
    std::vector<CombatVfxEventWireV1> validE;
    validE.reserve(std::min<std::size_t>(frame.events.size(), kMaxCombatVfxEventsV1));
    for (const auto& e : frame.events) {
        if (validE.size() >= kMaxCombatVfxEventsV1) break;
        if (validCombatVfxEventV1(e)) validE.push_back(e);
    }
    CombatVfxFrameFileHeadV1 head{};
    head.simTime = std::isfinite(frame.simTime) ? frame.simTime : 0.0;
    head.projectileCount = static_cast<std::uint32_t>(validP.size());
    head.decoyCount = static_cast<std::uint32_t>(validD.size());
    head.eventCount = static_cast<std::uint32_t>(validE.size());
    head.reserved = 0;
    if (std::fwrite(&head, sizeof head, 1, file_) != 1) return false;
    if (!validP.empty() && std::fwrite(validP.data(), sizeof(ProjectileWireV1), validP.size(), file_) != validP.size()) return false;
    if (!validD.empty() && std::fwrite(validD.data(), sizeof(DecoyWireV1), validD.size(), file_) != validD.size()) return false;
    if (!validE.empty() && std::fwrite(validE.data(), sizeof(CombatVfxEventWireV1), validE.size(), file_) != validE.size()) return false;
    return true;
}

void CombatVfxRecorderV1::close() {
    if (!file_) return;
    std::fflush(file_);
    std::fclose(file_);
    file_ = nullptr;
}

bool CombatVfxReplayV1::load(const std::filesystem::path& path, std::string* error) {
    frames_.clear();
    std::FILE* f = std::fopen(path.string().c_str(), "rb");
    if (!f) {
        if (error) *error = "cannot open combat VFX sidecar";
        return false;
    }
    char magic[6] = {};
    std::uint16_t version = 0;
    if (std::fread(magic, 1, sizeof magic, f) != sizeof magic ||
        std::memcmp(magic, kCombatVfxRecordMagicV1, sizeof magic) != 0 ||
        std::fread(&version, sizeof version, 1, f) != 1 || version != kCombatVfxVersionV1) {
        std::fclose(f);
        if (error) *error = "invalid combat VFX sidecar header";
        return false;
    }
    for (;;) {
        CombatVfxFrameFileHeadV1 head{};
        const std::size_t got = std::fread(&head, sizeof head, 1, f);
        if (got == 0) break;
        if (got != 1 || !std::isfinite(head.simTime) || head.projectileCount > kMaxProjectilesV1 ||
            head.decoyCount > kMaxDecoysV1 || head.eventCount > kMaxCombatVfxEventsV1 || head.reserved != 0u) {
            std::fclose(f);
            frames_.clear();
            if (error) *error = "corrupt combat VFX frame header";
            return false;
        }
        CombatVfxFrameV1 frame;
        frame.simTime = head.simTime;
        frame.projectiles.resize(head.projectileCount);
        if (head.projectileCount &&
            std::fread(frame.projectiles.data(), sizeof(ProjectileWireV1), head.projectileCount, f) != head.projectileCount) {
            std::fclose(f);
            frames_.clear();
            if (error) *error = "truncated combat VFX projectile records";
            return false;
        }
        frame.decoys.resize(head.decoyCount);
        if (head.decoyCount &&
            std::fread(frame.decoys.data(), sizeof(DecoyWireV1), head.decoyCount, f) != head.decoyCount) {
            std::fclose(f);
            frames_.clear();
            if (error) *error = "truncated combat VFX decoy records";
            return false;
        }
        frame.events.resize(head.eventCount);
        if (head.eventCount &&
            std::fread(frame.events.data(), sizeof(CombatVfxEventWireV1), head.eventCount, f) != head.eventCount) {
            std::fclose(f);
            frames_.clear();
            if (error) *error = "truncated combat VFX event records";
            return false;
        }
        for (const auto& p : frame.projectiles) {
            if (!validProjectileWireV1(p)) {
                std::fclose(f);
                frames_.clear();
                if (error) *error = "invalid projectile wire record";
                return false;
            }
        }
        for (const auto& d : frame.decoys) {
            if (!validDecoyWireV1(d)) {
                std::fclose(f);
                frames_.clear();
                if (error) *error = "invalid decoy wire record";
                return false;
            }
        }
        for (const auto& e : frame.events) {
            if (!validCombatVfxEventV1(e)) {
                std::fclose(f);
                frames_.clear();
                if (error) *error = "invalid combat VFX event record";
                return false;
            }
        }
        frames_.push_back(std::move(frame));
    }
    std::fclose(f);
    return true;
}

bool CombatVfxReplayV1::atTime(double simTimeS, CombatVfxFrameV1& out, bool interpolate) const {
    out.projectiles.clear();
    out.decoys.clear();
    out.events.clear();
    if (frames_.empty() || !std::isfinite(simTimeS)) return false;

    const auto it = std::upper_bound(frames_.begin(), frames_.end(), simTimeS,
                                     [](double t, const CombatVfxFrameV1& f) { return t < f.simTime; });
    if (it == frames_.begin()) {
        if (simTimeS < frames_.front().simTime - 0.05) {
            out.simTime = simTimeS;
            return true;
        }
        out = frames_.front();
        return true;
    }
    const std::size_t idx0 = static_cast<std::size_t>(it - frames_.begin() - 1);
    const CombatVfxFrameV1& f0 = frames_[idx0];
    if (!interpolate || it == frames_.end()) {
        out = f0;
        out.simTime = simTimeS;
        return true;
    }
    const CombatVfxFrameV1& f1 = *it;
    const double span = f1.simTime - f0.simTime;
    if (!(span > 1e-9) || span > 0.5) {
        out = f0;
        out.simTime = simTimeS;
        return true;
    }
    const double alpha = std::clamp((simTimeS - f0.simTime) / span, 0.0, 1.0);
    const float fa = static_cast<float>(alpha);
    out.simTime = simTimeS;

    // Linearly interpolate matching-ID projectiles; extrapolate briefly only for newly departing rounds inside [t0, t1).
    std::unordered_map<std::uint32_t, const ProjectileWireV1*> nextProj;
    nextProj.reserve(f1.projectiles.size() * 2u + 1u);
    for (const auto& p : f1.projectiles) nextProj[p.id] = &p;
    out.projectiles.reserve(f0.projectiles.size());
    for (const auto& p0 : f0.projectiles) {
        ProjectileWireV1 p = p0;
        if (const auto mit = nextProj.find(p0.id); mit != nextProj.end()) {
            const ProjectileWireV1& p1 = *mit->second;
            for (int k = 0; k < 3; ++k) {
                p.positionEcef[k] = p0.positionEcef[k] + (p1.positionEcef[k] - p0.positionEcef[k]) * alpha;
                p.velocityEcef[k] = p0.velocityEcef[k] + (p1.velocityEcef[k] - p0.velocityEcef[k]) * alpha;
            }
            p.ageS = p0.ageS + (p1.ageS - p0.ageS) * fa;
        } else {
            const double dt = simTimeS - f0.simTime;
            for (int k = 0; k < 3; ++k) p.positionEcef[k] = p0.positionEcef[k] + p0.velocityEcef[k] * dt;
            p.ageS = p0.ageS + static_cast<float>(dt);
        }
        out.projectiles.push_back(p);
    }

    // Linearly interpolate matching-ID decoys (flares & chaff).
    std::unordered_map<std::uint32_t, const DecoyWireV1*> nextDecoy;
    nextDecoy.reserve(f1.decoys.size() * 2u + 1u);
    for (const auto& d : f1.decoys) nextDecoy[d.id] = &d;
    out.decoys.reserve(f0.decoys.size());
    for (const auto& d0 : f0.decoys) {
        DecoyWireV1 d = d0;
        if (const auto mit = nextDecoy.find(d0.id); mit != nextDecoy.end()) {
            const DecoyWireV1& d1 = *mit->second;
            for (int k = 0; k < 3; ++k) {
                d.positionEcef[k] = d0.positionEcef[k] + (d1.positionEcef[k] - d0.positionEcef[k]) * alpha;
                d.velocityEcef[k] = d0.velocityEcef[k] + (d1.velocityEcef[k] - d0.velocityEcef[k]) * alpha;
            }
            d.ageS = d0.ageS + (d1.ageS - d0.ageS) * fa;
            d.intensityOrSigma = d0.intensityOrSigma + (d1.intensityOrSigma - d0.intensityOrSigma) * fa;
            d.bloomRadiusM = d0.bloomRadiusM + (d1.bloomRadiusM - d0.bloomRadiusM) * fa;
        } else {
            const double dt = simTimeS - f0.simTime;
            for (int k = 0; k < 3; ++k) d.positionEcef[k] = d0.positionEcef[k] + d0.velocityEcef[k] * dt;
            d.ageS = d0.ageS + static_cast<float>(dt);
        }
        out.decoys.push_back(d);
    }

    // Gather discrete combat events in [simTimeS - 0.12, simTimeS], deduplicated by serial.
    std::unordered_set<std::uint32_t> seenSerials;
    const double winLo = simTimeS - 0.12;
    std::size_t startIdx = idx0;
    while (startIdx > 0 && frames_[startIdx - 1].simTime >= winLo) --startIdx;
    for (std::size_t i = startIdx; i <= idx0; ++i) {
        for (const auto& ev : frames_[i].events) {
            if (seenSerials.insert(ev.serial).second) {
                out.events.push_back(ev);
                if (out.events.size() >= kMaxCombatVfxEventsV1) break;
            }
        }
        if (out.events.size() >= kMaxCombatVfxEventsV1) break;
    }
    return true;
}

} // namespace fsim::ipc
