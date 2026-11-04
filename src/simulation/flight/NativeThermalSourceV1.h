#pragma once
#include <array>

namespace fsim::sim {
/// Optional additive native thermal-state read; no FlightModel/public snapshot
/// layout or vtable change. Absence remains unavailable, never a zero-K engine.
enum class NativeGasTemperatureKindV1 : unsigned {Unavailable=0,ExhaustGas=1,InterTurbine=2};
struct NativeThermalSnapshotV1 {
    bool ambientAvailable=false;
    double ambientTemperatureK=0;
    unsigned engineCount=0;
    std::array<double,32> engineGasTemperatureK{};
    std::array<NativeGasTemperatureKindV1,32> engineGasTemperatureKind{};
};
class NativeThermalSourceV1 {
public:
    virtual ~NativeThermalSourceV1()=default;
    virtual bool nativeThermalSnapshotV1(NativeThermalSnapshotV1&) const noexcept=0;
};
} // namespace fsim::sim
