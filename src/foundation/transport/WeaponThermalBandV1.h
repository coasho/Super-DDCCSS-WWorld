#pragma once
#include "foundation/platform/SharedMemory.h"
#include <cstdint>
#include <string>

namespace fsim::ipc {
struct WeaponThermalBandAvailabilityV1 {
    std::uint64_t worldFrame=0,generation=0;
    std::uint32_t entity=0,bandCount=0;
};
/// Additive display demand only. No existing world/diagnostic frame changes,
/// sensor calls, simulation steps or actor-visible methods are involved.
class WeaponThermalBandV1 {
public:
    WeaponThermalBandV1()=default;
    ~WeaponThermalBandV1();
    WeaponThermalBandV1(const WeaponThermalBandV1&)=delete;
    WeaponThermalBandV1& operator=(const WeaponThermalBandV1&)=delete;
    bool create(const std::string&,std::uint64_t pid,std::uint32_t incarnation);
    bool open(const std::string&,std::uint64_t pid,std::uint32_t incarnation);
    void close() noexcept;
    bool valid() const noexcept;
    bool publish(std::uint64_t frame,std::uint32_t entity,std::uint64_t generation,unsigned bandCount) noexcept;
    bool availability(std::uint64_t currentFrame,WeaponThermalBandAvailabilityV1&) const noexcept;
    bool request(std::uint64_t frame,std::uint32_t entity,std::uint64_t generation,unsigned band) noexcept;
    bool selection(std::uint64_t currentFrame,std::uint32_t entity,std::uint64_t generation,unsigned& band) const noexcept;
private:
    struct Shared;
    platform::SharedMemory memory_;
    Shared* shared_=nullptr;
    std::uint64_t pid_=0,lastFrame_=0;
    std::uint32_t incarnation_=0;
    bool owner_=false;
};
}
