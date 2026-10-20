#include "foundation/transport/WeaponControlsSegment.h"
#include <new>

namespace fsim::ipc {
namespace {
std::string segmentName(const std::string& world) { return "fsim.weapon-controls.v1."+world; }
std::size_t stride(std::uint32_t capacity) { return sizeof(WeaponControlsSlotV1)+sizeof(WeaponControlsWireV1)*capacity; }
std::size_t bytes(std::uint32_t capacity) { return sizeof(WeaponControlsHeaderV1)+3*stride(capacity); }
}
WeaponControlsSegment::~WeaponControlsSegment() { close(); }
void WeaponControlsSegment::close() noexcept {
    if(owner_ && header_) header_->alive.store(0,std::memory_order_release);
    header_=nullptr; owner_=false; memory_.close();
}
WeaponControlsSlotV1* WeaponControlsSegment::slot(std::uint64_t sequence) const noexcept {
    return reinterpret_cast<WeaponControlsSlotV1*>(static_cast<unsigned char*>(const_cast<void*>(memory_.data()))+
        sizeof(WeaponControlsHeaderV1)+(sequence%3)*stride(capacity_));
}
bool WeaponControlsSegment::create(const std::string& world,std::uint32_t capacity,std::uint64_t pid,std::uint32_t incarnation) {
    close();
    if(!capacity || capacity>65536 || !pid || !incarnation || !memory_.create(segmentName(world),bytes(capacity))) return false;
    header_=static_cast<WeaponControlsHeaderV1*>(memory_.data());
    header_->magic=0;
    new(header_) WeaponControlsHeaderV1{};
    header_->capacity=capacity; header_->publisherPid=pid; header_->incarnation=incarnation;
    capacity_=capacity; pid_=pid; incarnation_=incarnation; owner_=true;
    for(std::uint64_t i=0;i<3;++i) new(slot(i)) WeaponControlsSlotV1{};
    header_->alive.store(1,std::memory_order_release); header_->magic=kWeaponControlsMagic;
    return true;
}
bool WeaponControlsSegment::open(const std::string& world,std::uint64_t pid,std::uint32_t incarnation) {
    close();
    if(!memory_.open(segmentName(world),sizeof(WeaponControlsHeaderV1))) return false;
    auto* h=static_cast<WeaponControlsHeaderV1*>(memory_.data());
    if(h->magic!=kWeaponControlsMagic || h->version!=1 || !h->capacity || h->capacity>65536 ||
       h->publisherPid!=pid || h->incarnation!=incarnation || !h->alive.load(std::memory_order_acquire)) { close(); return false; }
    const auto capacity=h->capacity;
    memory_.close();
    if(!memory_.open(segmentName(world),bytes(capacity))) return false;
    h=static_cast<WeaponControlsHeaderV1*>(memory_.data());
    if(h->magic!=kWeaponControlsMagic || h->version!=1 || h->capacity!=capacity || h->publisherPid!=pid ||
       h->incarnation!=incarnation || !h->alive.load(std::memory_order_acquire)) { close(); return false; }
    header_=h; capacity_=capacity; pid_=pid; incarnation_=incarnation;
    return true;
}
bool WeaponControlsSegment::publish(std::uint64_t sequence,const std::vector<ms::WeaponControlSnapshotV1>& samples) noexcept {
    if(!owner_ || !header_ || !sequence || samples.size()>capacity_) return false;
    auto* s=slot(sequence);
    s->seq.fetch_add(1,std::memory_order_acq_rel); s->sequence=0; s->count=0;
    auto* rows=reinterpret_cast<WeaponControlsWireV1*>(s+1);
    bool valid=true;
    for(std::size_t i=0;i<samples.size();++i) {
        if(samples[i].slot>=capacity_ || !encodeWeaponControls(samples[i],rows[i])) { valid=false; break; }
        for(std::size_t j=0;j<i;++j) if(samples[j].slot==samples[i].slot) valid=false;
        if(!valid) break;
    }
    if(valid) { s->count=static_cast<std::uint32_t>(samples.size()); s->sequence=sequence; }
    s->seq.fetch_add(1,std::memory_order_release);
    return valid;
}
bool WeaponControlsSegment::read(std::uint64_t sequence,std::vector<ms::WeaponControlSnapshotV1>& out) const {
    out.clear();
    if(!header_ || !sequence || header_->magic!=kWeaponControlsMagic || header_->publisherPid!=pid_ ||
       header_->incarnation!=incarnation_ || !header_->alive.load(std::memory_order_acquire)) return false;
    for(unsigned attempt=0;attempt<8;++attempt) {
        auto* s=slot(sequence); const auto before=s->seq.load(std::memory_order_acquire);
        if(before&1u) continue;
        if(s->sequence!=sequence || s->count>capacity_) return false;
        const auto count=s->count;
        const auto* rows=reinterpret_cast<const WeaponControlsWireV1*>(s+1);
        std::vector<ms::WeaponControlSnapshotV1> pending; pending.reserve(count);
        bool valid=true;
        for(std::uint32_t i=0;i<count;++i) {
            ms::WeaponControlSnapshotV1 sample;
            if(!decodeWeaponControls(rows[i],sample) || sample.slot>=capacity_) { valid=false; break; }
            for(const auto& prior:pending) if(prior.slot==sample.slot) valid=false;
            pending.push_back(std::move(sample));
        }
        std::atomic_thread_fence(std::memory_order_acquire);
        if(s->seq.load(std::memory_order_acquire)!=before) continue;
        if(!valid || header_->publisherPid!=pid_ || header_->incarnation!=incarnation_ ||
           !header_->alive.load(std::memory_order_acquire)) return false;
        out.swap(pending); return true;
    }
    return false;
}
}
