#pragma once
#include "contracts/world_stream/WeaponControlsLayout.h"
#include "foundation/platform/SharedMemory.h"
#include <string>
#include <vector>

namespace fsim::ipc {
/// Optional exact-frame sidecar; WorldLayout v9 remains byte-for-byte unchanged.
class WeaponControlsSegment {
public:
    ~WeaponControlsSegment();
    bool create(const std::string& world,std::uint32_t capacity,std::uint64_t pid,std::uint32_t incarnation);
    bool open(const std::string& world,std::uint64_t pid,std::uint32_t incarnation);
    void close() noexcept;
    bool publish(std::uint64_t sequence,const std::vector<ms::WeaponControlSnapshotV1>& samples) noexcept;
    bool read(std::uint64_t sequence,std::vector<ms::WeaponControlSnapshotV1>& out) const;
private:
    platform::SharedMemory memory_;
    WeaponControlsHeaderV1* header_=nullptr;
    std::uint64_t pid_=0;
    std::uint32_t incarnation_=0,capacity_=0;
    bool owner_=false;
    WeaponControlsSlotV1* slot(std::uint64_t sequence) const noexcept;
};
}
