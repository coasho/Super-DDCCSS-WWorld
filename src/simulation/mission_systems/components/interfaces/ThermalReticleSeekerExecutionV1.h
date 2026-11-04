#pragma once
#include "ThermalSeekerExecutionV1.h"

namespace fsim::ms::components {
/// A distinct scalar sampled detector, not a quadrant/image provider alias.
/// Existing receiver/configuration/result layouts and vtables are unchanged.
struct ThermalReticleConfigV1 {
    unsigned version=0,periods=0,samplesPerScan=0,spatialSamples=0;
    double scanRadiusRad=0,stopRadiusRad=0,frequencyHz=0;
    ThermalImageConfigV1 detector;
};
struct ThermalReticleResultV1;
class ThermalReticleSeekerExecutionV1 {
public:
    virtual ~ThermalReticleSeekerExecutionV1()=default;
    virtual bool configureThermalReticleV1(const ThermalReticleConfigV1&,std::string&)=0;
    virtual SeekerResultV2 sampleThermalReticleV1(const SeekerFrameV2&,const ThermalSceneFrameV1&)=0;
    virtual const ThermalReticleResultV1* thermalReticleV1() const noexcept=0;
};
}
