#pragma once

#include "simulation/mission_systems/weapon/Guidance.h"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>
#include <array>

namespace fsim::ms::components {

// Private execution contract. Samples contain observables, never target IDs,
// aircraft/decoy labels or a hidden Cartesian target velocity.
struct ImageExtentV2 { double widthRad=0, heightRad=0; };
/// Scene-physics inputs, not terminal observables. Never used by association.
/// Excess losses exclude free-space spreading, which the receiver computes.
struct SeekerSceneV3 {
    double rangeM=0, rcsM2=0, receiverExcessLossDb=0;
    double transmitterRangeM=0, transmitterPowerW=0, transmitterGainLinear=0;
    double transmitterExcessLossDb=0, transmitterFrequencyHz=0;
    double temperatureK=0; ///< Zero uses the admitted engineering source temperature.
};
struct SeekerSampleV2 {
    weapon::V3 direction;
    double signal=0;
    std::optional<double> rangeM, rangeRateMs;
    std::optional<ImageExtentV2> imageExtent;
    std::uint32_t illuminator=0;
    bool transmitterPathClear=false, receiverPathClear=false;
    std::optional<SeekerSceneV3> scene;
};
struct LauncherEstimateV2 {
    weapon::V3 position, velocity;
    double sampleTimeS=0;
};
struct SeekerFrameV2 {
    double timeS=0, ageS=0, dtS=0;
    weapon::V3 position, velocity, up, boresight;
    bool supported=false, illuminated=false;
    std::uint32_t configuredIlluminator=0;
    std::optional<LauncherEstimateV2> launcherEstimate;
    std::vector<SeekerSampleV2> samples;
    std::uint64_t receiverSeed=0; ///< Stable per-release stream identity.
    std::optional<std::array<weapon::V3,3>> bodyAxes; ///< ECEF body x/y/z axes.
};
struct SeekerTrackV2 {
    std::uint32_t localId=0;
    double sampleTimeS=0;
    weapon::V3 direction, directionRate;
    bool rateAvailable=false;
    std::optional<double> rangeM, rangeRateMs;
    std::optional<ImageExtentV2> imageExtent;
};
struct SeekerResultV2 {
    bool valid=false, active=false, tracking=false, supported=false;
    // Modeled measurement covariance availability is separate from hardware
    // calibration. Spatial availability means admitted extent features exist,
    // not that calibrated pixel imagery or a proprietary classifier exists.
    bool covarianceAvailable=false, spatialFeaturesAvailable=false;
    std::optional<SeekerTrackV2> track;
    std::optional<LauncherEstimateV2> midcourseEstimate;
    std::optional<double> timeToActiveS;
    // Index into this sample snapshot: World alone maps it to referee truth.
    std::optional<std::size_t> attributionIndex;
    std::optional<double> measurementTimeS, deliveredTimeS, processingReadyTimeS;
    std::optional<double> angularVarianceRad2;
    weapon::V3 pointingDirection;
};
class SeekerExecutionV2 {
public:
    virtual ~SeekerExecutionV2()=default;
    virtual SeekerResultV2 sample(const SeekerFrameV2&)=0;
    virtual void seedTrack(const SeekerTrackV2&)=0;
    virtual void resetTracking() noexcept=0;
};
}
