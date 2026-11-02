#pragma once
#include "contracts/world_stream/FusionFrameV1.h"
#include "foundation/platform/SharedMemory.h"
#include <memory>
#include <string>
#include <vector>

namespace fsim::ipc {
class FusionSegment {
public:
    ~FusionSegment();
    bool create(const std::string& world,std::uint32_t capacity,std::uint64_t pid,std::uint32_t incarnation);
    bool open(const std::string& world,std::uint64_t pid,std::uint32_t incarnation);
    void close() noexcept;
    void publish(std::uint64_t sequence,const std::vector<std::unique_ptr<FusionRecordV1>>& rows) noexcept;
    bool read(std::uint64_t sequence,std::uint32_t index,FusionRecordV1& out) const noexcept;
private:
    FusionSlotV1* slot(std::uint64_t sequence,std::uint32_t index) const noexcept;
    platform::SharedMemory memory_;
    FusionHeaderV1* header_=nullptr;
    std::uint32_t capacity_=0,incarnation_=0;
    std::uint64_t pid_=0;
    bool owner_=false;
};
}
