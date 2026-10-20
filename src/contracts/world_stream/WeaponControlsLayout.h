#pragma once
#include "fsim/WeaponControls.h"
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace fsim::ipc {
// Transport capacity, not an equipment or actuator parameter. Never truncate.
constexpr std::uint32_t kWeaponControlMaxFins=32;
constexpr std::uint32_t kWeaponControlsMagic=0x46494e31;
constexpr std::uint32_t kWeaponControlsVersion=1;
struct WeaponFinWireV1 {
    std::uint32_t id=0,index=0,flags=0,reserved=0;
    double actual=0,demand=0,minimum=0,maximum=0,rate=0;
};
struct WeaponControlsWireV1 {
    std::uint32_t entity=0,slot=0;
    std::uint64_t generation=0;
    double sampleTime=0,controllerTime=0;
    std::uint32_t flags=0,count=0;
    double commands[3]{};
    WeaponFinWireV1 fins[kWeaponControlMaxFins];
};
static_assert(sizeof(WeaponFinWireV1)==56 && sizeof(WeaponControlsWireV1)==64+56*kWeaponControlMaxFins,
              "fin wire records have explicit fields and no padding");
inline bool encodeWeaponControls(const ms::WeaponControlSnapshotV1& in,WeaponControlsWireV1& out) {
    out={};
    if(!in.entity || !in.generation || !std::isfinite(in.sampleTimeS) || in.sampleTimeS<0 ||
       in.fins.size()>kWeaponControlMaxFins || std::isinf(in.controllerTimeS)) return false;
    out.entity=in.entity; out.slot=in.slot; out.generation=in.generation;
    out.sampleTime=in.sampleTimeS; out.controllerTime=in.controllerTimeS;
    out.flags=(in.available?1u:0u)|(in.controllerAvailable?2u:0u);
    out.count=static_cast<std::uint32_t>(in.fins.size());
    for(std::size_t i=0;i<3;++i) { if(!std::isfinite(in.commands[i])) return false; out.commands[i]=in.commands[i]; }
    for(std::size_t i=0;i<in.fins.size();++i) {
        const auto& f=in.fins[i]; auto& w=out.fins[i];
        if(!f.id) return false;
        for(std::size_t j=0;j<i;++j) if(in.fins[j].id==f.id) return false;
        if(f.valid && (!std::isfinite(f.actualRad) || !std::isfinite(f.demandRad) ||
           !std::isfinite(f.minimumRad) || !std::isfinite(f.maximumRad) || !std::isfinite(f.rateRadS) ||
           f.minimumRad>=f.maximumRad || f.rateRadS<=0)) return false;
        w.id=f.id; w.index=f.nativeIndex;
        w.flags=(f.valid?1u:0u)|(f.failedStuck?2u:0u)|(f.failedZero?4u:0u)|(f.failedHardover?8u:0u);
        w.actual=f.actualRad; w.demand=f.demandRad; w.minimum=f.minimumRad; w.maximum=f.maximumRad; w.rate=f.rateRadS;
    }
    return true;
}
inline bool decodeWeaponControls(const WeaponControlsWireV1& in,ms::WeaponControlSnapshotV1& out) {
    out={};
    if(in.count>kWeaponControlMaxFins || (in.flags&~3u)) return false;
    out.entity=in.entity; out.slot=in.slot; out.generation=in.generation;
    out.sampleTimeS=in.sampleTime; out.controllerTimeS=in.controllerTime;
    out.available=(in.flags&1)!=0; out.controllerAvailable=(in.flags&2)!=0;
    for(std::size_t i=0;i<3;++i) out.commands[i]=in.commands[i];
    for(std::uint32_t i=0;i<in.count;++i) {
        const auto& w=in.fins[i]; if(w.reserved || (w.flags&~15u)) return false;
        ms::WeaponFinStateV1 f; f.id=w.id; f.nativeIndex=w.index;
        f.valid=(w.flags&1)!=0; f.failedStuck=(w.flags&2)!=0; f.failedZero=(w.flags&4)!=0; f.failedHardover=(w.flags&8)!=0;
        f.actualRad=w.actual; f.demandRad=w.demand; f.minimumRad=w.minimum; f.maximumRad=w.maximum; f.rateRadS=w.rate;
        out.fins.push_back(f);
    }
    WeaponControlsWireV1 checked;
    return encodeWeaponControls(out,checked);
}
struct alignas(64) WeaponControlsHeaderV1 {
    std::uint32_t magic=0,version=1,capacity=0,incarnation=0;
    std::uint64_t publisherPid=0;
    std::atomic<std::uint32_t> alive{0};
};
struct alignas(64) WeaponControlsSlotV1 {
    std::atomic<std::uint32_t> seq{0};
    std::uint32_t count=0;
    std::uint64_t sequence=0;
};
}
