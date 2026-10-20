#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

namespace fsim::ms {
/// Referee diagnostics, not a sensor report or an actor observation.
struct WeaponFinStateV1 {
    std::uint32_t id=0, nativeIndex=0;
    bool valid=false, failedStuck=false, failedZero=false, failedHardover=false;
    double actualRad=std::numeric_limits<double>::quiet_NaN();
    double demandRad=std::numeric_limits<double>::quiet_NaN();
    double minimumRad=std::numeric_limits<double>::quiet_NaN();
    double maximumRad=std::numeric_limits<double>::quiet_NaN();
    double rateRadS=std::numeric_limits<double>::quiet_NaN();
};
struct WeaponControlSnapshotV1 {
    std::uint32_t entity=0, slot=0;
    std::uint64_t generation=0;
    bool available=false, controllerAvailable=false;
    double sampleTimeS=0;
    double controllerTimeS=std::numeric_limits<double>::quiet_NaN();
    std::array<double,3> commands{}; // aileron, elevator, rudder; normalized
    std::vector<WeaponFinStateV1> fins;
};
}
