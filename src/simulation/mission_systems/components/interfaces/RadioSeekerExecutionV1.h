#pragma once

#include "SeekerExecutionV2.h"
#include "SeekerNavigationPoseV1.h"
#include "simulation/mission_systems/radar/Signal.h"
#include "simulation/mission_systems/radar/SignalDiagnosticsV1.h"
#include "simulation/mission_systems/radar/IlluminationClockV1.h"
#include "simulation/mission_systems/rf/Antenna.h"
#include <string>

namespace fsim::ms::components {

/// Explicit private pulse-domain-equivalent receiver selection. Registration
/// resolves all policy fields; existing scalar V3/V2 paths do not call it.
struct RadioReceiverConfigV1 {
    unsigned version=0;
    double frequencyHz=0,temperatureK=0,noiseFigureDb=0,systemLossDb=0;
    double transmitAveragePowerW=0,bandwidthHz=0,monopulseSlope=0;
    double clutterRateSigmaMs=0,associationGateRad=0;
    double temporalMemoryS=0; ///< Existing explicit finite track-memory policy, not an implicit new coefficient.
    double observationWindowS=0; ///< Explicit receiver_rf_dwell_s, independent of illuminator dwell duration.
    rf::ApertureSpec aperture;
    radar::signal::Waveform waveform;
    radar::signal::Settings processing;
};
/// Scene physics only. Doppler is the ACTUAL echo frequency in Hz, positive
/// approaching. delayRangeM is c*propagation_delay/2: monostatic R, or the
/// declared bistatic path-delay equivalent. Neither becomes passive range.
struct RadioEchoSceneV1 {
    weapon::V3 direction;
    double receiverRangeM=0,transmitterRangeM=0,delayRangeM=0,dopplerHz=0;
    double rcsM2=0,receiverExcessLossDb=0,transmitterExcessLossDb=0;
    double transmitterPowerW=0,transmitterGainLinear=0;
    unsigned fluctuation=0;
};
struct RadioSceneFrameV1 {
    double timeS=0,frequencyHz=0,interferencePowerW=0;
    weapon::V3 up;
    std::optional<SeekerNavigationPoseV1> navigation;
    bool illuminationAvailable=false;
    std::optional<radar::signal::Waveform> illuminationWaveform;
    std::optional<radar::IlluminationClockV1> illuminationClock;
    std::optional<double> mainBeamClutterRateMs,altitudeM;
    std::vector<RadioEchoSceneV1> echoes,groundPatches;
    /// Only signal::Echo.truth scoring and a final post-detection metadata map
    /// read these. Detection/association may not inspect source identity.
    std::vector<std::optional<std::size_t>> refereeAttributionIndices;
};
struct RadioMeasurementV1 {
    weapon::V3 direction;
    double signalToNoise=0,angularVarianceRad2=0;
    std::optional<double> rangeM,rangeRateMs;
    std::optional<std::size_t> refereeAttributionIndex;
    bool navigationReconstructed=false; ///< Temporal bearings retain their ORIGINAL estimated attitude reconstruction.
};
struct RadioReceiverResultV1 {
    bool valid=false,semiActive=false;
    double timeS=0;
    double observationStartTimeS=0,observationEndTimeS=0,integratedTimeS=0;
    double coincidenceFirstTimeS=0,coincidenceLastTimeS=0;
    unsigned coincidenceIntervals=0;
    std::vector<unsigned> programIntervals;
    std::vector<double> intervalStartTimeS,intervalEndTimeS,intervalEnergyFraction;
    weapon::V3 pointing;
    radar::signal::Waveform waveform;
    RadioReceiverConfigV1 config;
    radar::signal::Stats stats;
    radar::signal::Snapshot spectrum;
    radar::signal::CellDiagnosticsV1 cells;
    std::vector<radar::signal::Hit> hits;
    std::vector<RadioMeasurementV1> measurements;
};
class RadioSeekerExecutionV1 {
public:
    virtual ~RadioSeekerExecutionV1()=default;
    virtual bool configureRadioV1(const RadioReceiverConfigV1&,std::string&)=0;
    virtual SeekerResultV2 sampleRadioV1(const SeekerFrameV2&,const RadioSceneFrameV1&)=0;
    virtual const RadioReceiverResultV1* radioSpectrumV1() const noexcept=0;
    virtual const RadioReceiverConfigV1* radioPolicyV1() const noexcept=0;
};
} // namespace fsim::ms::components
