#pragma once

#include "SeekerExecutionV2.h"
#include "SeekerNavigationPoseV1.h"
#include <string>

namespace fsim::ms::components {

/// Additive private optical detector policy. Zero initialization is invalid;
/// registration must resolve every coefficient from an explicit versioned card.
struct ThermalImageConfigV1 {
    unsigned version=0, width=0, height=0, adcBits=0, exposureSamples=0;
    double pixelPitchRad=0, psfSigmaRad=0, apertureAreaM2=0;
    double opticalTransmission=0, quantumEfficiency=0, effectiveWavelengthM=0;
    double integrationTimeS=0, pixelResponseTimeS=0;
    double readNoiseElectrons=0, darkCurrentElectronsS=0;
    double fullWellElectrons=0, detectionSigma=0, backgroundRadiusPixels=0;
    double minimumFeaturePixels=0, associationGateRad=0;
};

/// Scene formation inputs only; none are target classifications or identities.
/// Radiance is band-integrated W/(m^2 sr). Rectangular support is an actual
/// projected emitting/occluding patch, not an association feature. Several
/// patches can describe one body/exhaust. Direction rate is ECEF unit-ray /s;
/// finite exposure uses a declared constant-rate backward scene approximation.
struct ThermalRadiancePatchV1 {
    weapon::V3 direction, directionRate;
    double widthRad=0, heightRad=0, rangeM=0;
    double radianceWm2Sr=0, transmission=0, pathRadianceWm2Sr=0;
};
struct ThermalSceneFrameV1 {
    double timeS=0, skyRadianceWm2Sr=0, groundRadianceWm2Sr=0;
    double groundHorizonCosine=0; ///< Native horizon: ground when dot(ray,up) is below this value.
    weapon::V3 up;
    std::optional<SeekerNavigationPoseV1> navigation;
    std::vector<ThermalRadiancePatchV1> patches;
    /// Referee-only bookkeeping, same length as patches. Neither image
    /// formation, detector, component association nor tracking reads these.
    std::vector<std::optional<std::size_t>> refereeAttributionIndices;
};

/// A sibling interface: existing public and private seeker vtables/layouts
/// retain their meaning. Only an explicit new optical binding calls this path.
struct ThermalImageResultV1;
class ThermalSeekerExecutionV1 {
public:
    virtual ~ThermalSeekerExecutionV1()=default;
    virtual bool configureThermalV1(const ThermalImageConfigV1&,std::string&)=0;
    virtual SeekerResultV2 sampleThermalV1(const SeekerFrameV2&,const ThermalSceneFrameV1&)=0;
    virtual const ThermalImageResultV1* thermalImageV1() const noexcept=0;
    virtual const ThermalImageConfigV1* thermalPolicyV1() const noexcept=0;
};

} // namespace fsim::ms::components
