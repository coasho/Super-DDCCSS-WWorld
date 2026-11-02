#pragma once
// Additive observational sidecar. WorldLayout v9 and public SDK layouts stay frozen.
#include <atomic>
#include <cmath>
#include <cstdint>
#include <type_traits>

namespace fsim::ipc {
constexpr std::uint32_t kFusionMagicV1=0x46555331u;
constexpr std::uint32_t kFusionDisplayEntitiesV1=128;
struct FusionEntityWireV1 {
    std::uint64_t id=0;
    std::uint32_t state=0,sourceCount=0,sourceKinds=0,identityKnown=0,standardIdentity=0,reserved=0;
    double lastUpdateS=0,validUntilS=0;
    double stateEcef[6]{},covarianceEcef[36]{};
};
struct FusionRecordV1 {
    std::uint32_t vehicleId=0,count=0,total=0,reserved=0;
    std::uint64_t generation=0,rejectedInputs=0,capacityRefusals=0;
    double timeS=0;
    FusionEntityWireV1 entities[kFusionDisplayEntitiesV1];
};
struct alignas(64) FusionHeaderV1 {
    std::uint32_t magic=0,version=1,capacity=0,incarnation=0;
    std::uint64_t publisherPid=0;
    std::atomic<std::uint32_t> alive{0};
};
struct alignas(64) FusionSlotV1 {
    std::atomic<std::uint32_t> seq{0};
    std::uint32_t reserved=0;
    std::uint64_t sequence=0;
    FusionRecordV1 record;
};
inline bool validFusionRecordV1(const FusionRecordV1& r) noexcept {
    if(!r.vehicleId||!r.generation||r.reserved||!std::isfinite(r.timeS)||r.timeS<0||
       r.count>kFusionDisplayEntitiesV1||r.count>r.total)return false;
    for(std::uint32_t i=0;i<r.count;++i) {
        const auto& e=r.entities[i];
        if(!e.id||e.state>2||e.identityKnown>1||e.reserved||(e.sourceKinds&~3u)||
           !std::isfinite(e.lastUpdateS)||e.lastUpdateS>r.timeS||!std::isfinite(e.validUntilS))return false;
        for(std::uint32_t j=0;j<i;++j)if(r.entities[j].id==e.id)return false;
        for(double v:e.stateEcef)if(!std::isfinite(v))return false;
        for(double v:e.covarianceEcef)if(!std::isfinite(v))return false;
    }
    return true;
}
static_assert(std::is_trivially_copyable<FusionRecordV1>::value,"fusion wire is plain immutable snapshot data");
static_assert(sizeof(FusionEntityWireV1)==384,"fusion V1 entity stride is frozen");
}
