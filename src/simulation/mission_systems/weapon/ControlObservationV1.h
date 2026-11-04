#pragma once

#include "Autopilot.h"
#include <cstdint>

namespace fsim::sim { class FlightModel; }
namespace fsim::ms::weapon {
struct OperatingPointInputV1;

// Private trusted tooling only. References are borrowed for this synchronous
// callback and must not be retained or mutated. No actor/public SDK access.
// Thread-local binding is not propagated to parallel workers; a caller uses
// the serial execution thread or installs its own trusted binding there.
// Inputs are the exact PRE-integration values used by the authoritative call;
// commands are its result, not a second calculation or force application.
struct ControlObservationV1 {
    const sim::FlightModel& model;
    std::uint32_t entity;
    std::uint64_t generation;
    double timeS, dtS;
    unsigned mechanismCode;
    const char* mechanism;
    bool dispatched;
    const AutopilotInput& input;
    const FinControlConfigV1& config;
    const ControlStopsV1& stops;
    const OperatingPointInputV1* operatingPoint;
    const FinCommands& commands;
};
struct ControlObserverV1 {
    const sim::FlightModel* model=nullptr;
    void* context=nullptr;
    void (*observe)(void*,const ControlObservationV1&) noexcept=nullptr;
};
inline thread_local ControlObserverV1 controlObserverV1;
class ScopedControlObserverV1 {
public:
    explicit ScopedControlObserverV1(ControlObserverV1 observer) noexcept
        : previous_(controlObserverV1) {controlObserverV1=observer;}
    ~ScopedControlObserverV1() {controlObserverV1=previous_;}
    ScopedControlObserverV1(const ScopedControlObserverV1&)=delete;
    ScopedControlObserverV1& operator=(const ScopedControlObserverV1&)=delete;
private:
    ControlObserverV1 previous_;
};
} // namespace fsim::ms::weapon
