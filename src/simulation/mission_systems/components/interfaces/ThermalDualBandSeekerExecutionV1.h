#pragma once
#include "ThermalSeekerExecutionV1.h"

namespace fsim::ms::components {
/// Additive private dual-band contract. No V1 policy, scene or vtable changes.
/// Each channel has its own optics, photon energy, detector, noise and scene
/// radiance. Spectral gates apply to measured radiant contrast, never identity.
struct ThermalDualBandConfigV1 {
    unsigned version=0;
    std::array<ThermalImageConfigV1,2> channels;
    double angularPairGateRad=0,logRatioGate=0,spectralMemoryS=0;
};
struct ThermalDualBandSceneFrameV1 {
    std::array<ThermalSceneFrameV1,2> channels;
};
struct ThermalDualBandResultV1;
class ThermalDualBandSeekerExecutionV1 {
public:
    virtual ~ThermalDualBandSeekerExecutionV1()=default;
    virtual bool configureThermalDualBandV1(const ThermalDualBandConfigV1&,std::string&)=0;
    virtual SeekerResultV2 sampleThermalDualBandV1(const SeekerFrameV2&,const ThermalDualBandSceneFrameV1&)=0;
    virtual const ThermalDualBandResultV1* thermalDualBandV1() const noexcept=0;
    virtual const ThermalDualBandConfigV1* thermalDualBandPolicyV1() const noexcept=0;
};
}
