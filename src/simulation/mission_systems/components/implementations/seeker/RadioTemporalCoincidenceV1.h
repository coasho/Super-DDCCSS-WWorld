#pragma once
#include "simulation/mission_systems/components/support/MeasuredSeekerV2.h"
#include "simulation/mission_systems/radar/Signal.h"
#include "simulation/mission_systems/radar/IlluminationClockV1.h"
#include <array>
#include <optional>
#include <vector>

namespace fsim::ms::components {
struct RadioPulseWindowV1 {
    radar::signal::Waveform waveform;
    std::vector<unsigned> programIntervals;
    std::vector<double> startTimeS,endTimeS,energyFraction;
    double observationStartTimeS=0,observationEndTimeS=0,integratedTimeS=0;
};
/// Complete transmitted pulses only; never moves a PRF switch into a shorter
/// listener window. At most the latest contiguous block per native interval
/// is processed (two separated blocks cannot form one coherent FFT).
std::optional<RadioPulseWindowV1> radioPulseWindowV1(const radar::signal::Waveform&,
    const radar::IlluminationClockV1&,double firstObservedS,double captureS,double windowS);

struct RadioTemporalPeakV1 {
    double foldedRangeM=0,foldedRateMs=0,snr=0,angularVarianceRad2=0;
    weapon::V3 direction; ///< Already reconstructed with THIS capture's navigation attitude.
    std::optional<std::size_t> refereeAttributionIndex; ///< Scoring only, never an association key.
};
struct RadioTemporalSegmentV1 {
    unsigned programInterval=0;
    double timeS=0,rateResolutionMs=0;
    radar::signal::Interval waveform;
    std::vector<RadioTemporalPeakV1> peaks;
};
struct RadioTemporalDetectionV1 {
    weapon::V3 direction,directionRate;
    double angularVarianceRad2=0,snr=0,firstObservationS=0,lastObservationS=0;
    unsigned intervals=0;
    /// Measured delay-equivalent range/rate aliases; diagnostic only for SARH.
    std::vector<std::array<double,2>> hypotheses;
    std::optional<std::size_t> refereeAttributionIndex; ///< From a current capture member only.
};
class RadioTemporalCoincidenceV1 {
public:
    void reset() noexcept;
    /// Keeps the latest measured coherent block per DISTINCT native PRF.
    /// Returns false on malformed inputs/capacity, without admitting them.
    bool update(double captureS,double memoryS,double angularGateRad,unsigned requiredM,
        const radar::signal::Settings&,double gateM,const std::vector<RadioTemporalSegmentV1>&,
        std::vector<RadioTemporalDetectionV1>&);
    std::size_t retainedSegments() const noexcept {return segments_.size();}
private:
    double lastTimeS_=-1;
    std::vector<RadioTemporalSegmentV1> segments_;
};
} // namespace fsim::ms::components
