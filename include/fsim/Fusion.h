#pragma once
// Additive V1 sensor/link-derived platform fusion reports. No referee truth.
#include <cstdint>
#include <vector>

namespace fsim::ms {

enum class FusionStateV1 : std::uint32_t { Live=0, Coasting=1, Lost=2 };
enum class FusionSourceKindV1 : std::uint32_t { LocalRadar=0, DeliveredLink=1 };
struct FusionSourceV1 {
    FusionSourceKindV1 kind=FusionSourceKindV1::LocalRadar;
    std::uint32_t system=0, payload=0;
    std::uint64_t track=0;
    double sampleTimeS=0,receivedTimeS=0;
    bool diagonalLinkCovariance=false;
};

struct FusedEntityV1 {
    std::uint64_t id=0;
    FusionStateV1 state=FusionStateV1::Lost;
    double timeS=0,lastUpdateS=0,validUntilS=0;
    double stateEcef[6]={};      ///< interleaved x,vx,y,vy,z,vz
    double covarianceEcef[36]={}; ///< row major, same interleaved order
    bool identityKnown=false;   ///< absence of sensor identity evidence remains unknown
    std::uint32_t standardIdentity=0;
    std::vector<FusionSourceV1> sources;
};

struct FusionStatusV1 {
    double timeS=0;
    std::uint64_t rejectedInputs=0,capacityRefusals=0;
    std::uint32_t entityCapacity=512,sourceCapacity=64;
};

} // namespace fsim::ms
