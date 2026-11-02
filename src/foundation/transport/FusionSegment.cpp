#include "foundation/transport/FusionSegment.h"
#include <cstring>
#include <new>

namespace fsim::ipc {
namespace {
std::string nameOf(const std::string& world){return "fsim.fusion.v1."+world;}
std::size_t bytes(std::uint32_t capacity){return sizeof(FusionHeaderV1)+3*static_cast<std::size_t>(capacity)*sizeof(FusionSlotV1);}
}
FusionSegment::~FusionSegment(){close();}
void FusionSegment::close() noexcept {
    if(owner_&&header_)header_->alive.store(0,std::memory_order_release);
    header_=nullptr;owner_=false;capacity_=0;memory_.close();
}
FusionSlotV1* FusionSegment::slot(std::uint64_t sequence,std::uint32_t index) const noexcept {
    return reinterpret_cast<FusionSlotV1*>(static_cast<unsigned char*>(const_cast<void*>(memory_.data()))+sizeof(FusionHeaderV1))+
        (sequence%3)*capacity_+index;
}
bool FusionSegment::create(const std::string& world,std::uint32_t capacity,std::uint64_t pid,std::uint32_t incarnation) {
    close();if(!capacity||capacity>65536||!pid||!incarnation||!memory_.create(nameOf(world),bytes(capacity)))return false;
    header_=new(memory_.data()) FusionHeaderV1{};
    capacity_=capacity;pid_=pid;incarnation_=incarnation;owner_=true;
    header_->capacity=capacity;header_->publisherPid=pid;header_->incarnation=incarnation;
    for(std::uint64_t s=0;s<3;++s)for(std::uint32_t i=0;i<capacity;++i){auto* row=slot(s,i);new(&row->seq)std::atomic<std::uint32_t>{0};row->sequence=0;}
    header_->alive.store(1,std::memory_order_release);header_->magic=kFusionMagicV1;return true;
}
bool FusionSegment::open(const std::string& world,std::uint64_t pid,std::uint32_t incarnation) {
    close();if(!memory_.open(nameOf(world),sizeof(FusionHeaderV1)))return false;
    const auto* h=static_cast<const FusionHeaderV1*>(memory_.data());
    auto valid=[&](const FusionHeaderV1* p){return p->magic==kFusionMagicV1&&p->version==1&&p->capacity&&p->capacity<=65536&&
        p->publisherPid==pid&&p->incarnation==incarnation&&p->alive.load(std::memory_order_acquire);};
    if(!valid(h)){close();return false;}
    const auto capacity=h->capacity;memory_.close();if(!memory_.open(nameOf(world),bytes(capacity)))return false;
    h=static_cast<const FusionHeaderV1*>(memory_.data());if(!valid(h)||h->capacity!=capacity){close();return false;}
    header_=static_cast<FusionHeaderV1*>(memory_.data());capacity_=capacity;pid_=pid;incarnation_=incarnation;return true;
}
void FusionSegment::publish(std::uint64_t sequence,const std::vector<std::unique_ptr<FusionRecordV1>>& rows) noexcept {
    if(!owner_||!header_||!sequence)return;
    for(std::uint32_t i=0;i<capacity_;++i) {
        auto* s=slot(sequence,i);s->seq.fetch_add(1,std::memory_order_acq_rel);s->sequence=0;
        if(i<rows.size()&&rows[i]&&validFusionRecordV1(*rows[i])){s->record=*rows[i];s->sequence=sequence;}
        s->seq.fetch_add(1,std::memory_order_release);
    }
}
bool FusionSegment::read(std::uint64_t sequence,std::uint32_t index,FusionRecordV1& out) const noexcept {
    out={};if(!header_||!sequence||index>=capacity_)return false;
    auto valid=[&](){return header_->magic==kFusionMagicV1&&header_->version==1&&header_->capacity==capacity_&&
        header_->publisherPid==pid_&&header_->incarnation==incarnation_&&header_->alive.load(std::memory_order_acquire);};
    if(!valid())return false;
    for(unsigned attempt=0;attempt<8;++attempt){
        const auto* s=slot(sequence,index);const auto before=s->seq.load(std::memory_order_acquire);if(before&1u)continue;
        const auto epoch=s->sequence;if(epoch!=sequence)return false;
        std::memcpy(&out,&s->record,sizeof(out));std::atomic_thread_fence(std::memory_order_acquire);
        if(s->seq.load(std::memory_order_acquire)!=before)continue;
        if(valid()&&validFusionRecordV1(out))return true;
        break;
    }
    out={};return false;
}
}
