#include "WeaponThermalBandV1.h"
#include <atomic>
#include <cstdio>
#include <limits>
#include <new>
#ifndef _WIN32
#include <sys/mman.h>
#endif

namespace fsim::ipc {
namespace {
constexpr std::uint32_t magic=0x57425431;
constexpr std::uint64_t maximumAge=300;
struct Packet {std::uint64_t frame=0,generation=0;std::uint32_t entity=0,value=0;};
struct Slot {
    std::atomic<std::uint64_t> guard{0},frame{0},generation{0};
    std::atomic<std::uint32_t> entity{0},value{0};
};
static_assert(std::atomic<std::uint64_t>::is_always_lock_free&&std::atomic<std::uint32_t>::is_always_lock_free);
bool write(Slot& slot,const Packet& packet) noexcept {
    for(unsigned attempt=0;attempt<8;++attempt) {
        auto before=slot.guard.load(std::memory_order_seq_cst);
        if((before&1)||before>std::numeric_limits<std::uint64_t>::max()-2)continue;
        if(!slot.guard.compare_exchange_weak(before,before+1,std::memory_order_seq_cst))continue;
        slot.frame.store(packet.frame,std::memory_order_seq_cst);slot.generation.store(packet.generation,std::memory_order_seq_cst);
        slot.entity.store(packet.entity,std::memory_order_seq_cst);slot.value.store(packet.value,std::memory_order_seq_cst);
        slot.guard.store(before+2,std::memory_order_seq_cst);return true;
    }
    return false;
}
bool read(const Slot& slot,Packet& packet) noexcept {
    for(unsigned attempt=0;attempt<8;++attempt) {
        const auto before=slot.guard.load(std::memory_order_seq_cst);if(!before||(before&1))continue;
        Packet next{slot.frame.load(std::memory_order_seq_cst),slot.generation.load(std::memory_order_seq_cst),
            slot.entity.load(std::memory_order_seq_cst),slot.value.load(std::memory_order_seq_cst)};
        if(before!=slot.guard.load(std::memory_order_seq_cst))continue;
        packet=next;return true;
    }
    return false;
}
bool fresh(std::uint64_t current,std::uint64_t source) {return source&&source<=current&&current-source<=maximumAge;}
bool validWorld(const std::string& world) {return !world.empty()&&world.size()<=160&&world.find_first_of("/\\")==std::string::npos&&world.find('\0')==std::string::npos;}
std::string segment(const std::string& world,std::uint64_t pid,std::uint32_t incarnation) {
    return "fsim.weapon-thermal-band.v1."+world+"."+std::to_string(pid)+"."+std::to_string(incarnation);
}
}
struct alignas(64) WeaponThermalBandV1::Shared {
    std::atomic<std::uint32_t> ready{0},magicValue{0},version{0},incarnation{0};
    std::atomic<std::uint64_t> pid{0};
    Slot availability,request;
};
WeaponThermalBandV1::~WeaponThermalBandV1(){close();}
void WeaponThermalBandV1::close() noexcept {
    if(owner_&&shared_&&shared_->pid.load(std::memory_order_acquire)==pid_&&shared_->incarnation.load(std::memory_order_acquire)==incarnation_)
        shared_->ready.store(0,std::memory_order_release);
#ifndef _WIN32
    if(owner_&&memory_.valid()) {
        char name[256];const int length=std::snprintf(name,sizeof(name),"/%s",memory_.name().c_str());
        if(length>0&&static_cast<std::size_t>(length)<sizeof(name))::shm_unlink(name);
    }
#endif
    shared_=nullptr;owner_=false;pid_=lastFrame_=0;incarnation_=0;memory_.close();
}
bool WeaponThermalBandV1::valid() const noexcept {
    return shared_&&shared_->ready.load(std::memory_order_acquire)==1&&
        shared_->magicValue.load(std::memory_order_relaxed)==magic&&shared_->version.load(std::memory_order_relaxed)==1&&
        shared_->pid.load(std::memory_order_relaxed)==pid_&&shared_->incarnation.load(std::memory_order_relaxed)==incarnation_&&platform::processAlive(pid_);
}
bool WeaponThermalBandV1::create(const std::string& world,std::uint64_t pid,std::uint32_t incarnation) {
    close();if(!validWorld(world)||pid!=platform::currentProcessId()||!incarnation)return false;
    if(!memory_.create(segment(world,pid,incarnation),sizeof(Shared)))return false;
    if(!memory_.created()){memory_.close();return false;}
    shared_=::new(memory_.data())Shared{};pid_=pid;incarnation_=incarnation;owner_=true;
    shared_->pid.store(pid,std::memory_order_relaxed);shared_->incarnation.store(incarnation,std::memory_order_relaxed);
    shared_->magicValue.store(magic,std::memory_order_relaxed);shared_->version.store(1,std::memory_order_relaxed);
    shared_->ready.store(1,std::memory_order_release);return true;
}
bool WeaponThermalBandV1::open(const std::string& world,std::uint64_t pid,std::uint32_t incarnation) {
    close();if(!validWorld(world)||!pid||!incarnation||!memory_.open(segment(world,pid,incarnation),sizeof(Shared)))return false;
    shared_=static_cast<Shared*>(memory_.data());pid_=pid;incarnation_=incarnation;
    if(!valid()){close();return false;}return true;
}
bool WeaponThermalBandV1::publish(std::uint64_t frame,std::uint32_t entity,std::uint64_t generation,unsigned bandCount) noexcept {
    if(!owner_||!valid()||!frame||frame<=lastFrame_||((entity==0)!=(generation==0))||bandCount>2||(!entity&&bandCount))return false;
    if(!write(shared_->availability,{frame,generation,entity,bandCount}))return false;
    lastFrame_=frame;return true;
}
bool WeaponThermalBandV1::availability(std::uint64_t current,WeaponThermalBandAvailabilityV1& out) const noexcept {
    if(!valid())return false;
    Packet packet;
    if(!read(shared_->availability,packet)||!fresh(current,packet.frame)||packet.value>2||((packet.entity==0)!=(packet.generation==0))||!valid())return false;
    out={packet.frame,packet.generation,packet.entity,packet.value};return true;
}
bool WeaponThermalBandV1::request(std::uint64_t frame,std::uint32_t entity,std::uint64_t generation,unsigned band) noexcept {
    if(owner_||!entity||!generation||band>=2||!valid())return false;
    WeaponThermalBandAvailabilityV1 current;
    if(!availability(frame,current)||current.worldFrame!=frame||current.entity!=entity||current.generation!=generation||band>=current.bandCount)return false;
    return write(shared_->request,{frame,generation,entity,band});
}
bool WeaponThermalBandV1::selection(std::uint64_t frame,std::uint32_t entity,std::uint64_t generation,unsigned& band) const noexcept {
    if(!owner_||!valid())return false;
    Packet request;
    WeaponThermalBandAvailabilityV1 current;
    if(!availability(frame,current)||current.entity!=entity||current.generation!=generation||
       !read(shared_->request,request)||!fresh(frame,request.frame)||request.entity!=entity||request.generation!=generation||request.value>=current.bandCount||!valid())return false;
    band=request.value;return true;
}
}
