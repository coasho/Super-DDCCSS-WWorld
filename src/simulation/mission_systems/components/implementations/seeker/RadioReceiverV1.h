#pragma once
#include "simulation/mission_systems/components/interfaces/RadioSeekerExecutionV1.h"
#include "RadioTemporalCoincidenceV1.h"

namespace fsim::ms::components {
class RadioReceiverV1 {
public:
    static bool validConfig(const RadioReceiverConfigV1&) noexcept;
    static bool validWaveform(const radar::signal::Waveform&,const radar::signal::Settings&) noexcept;
    static bool validObservedWaveform(const radar::signal::Waveform&,const radar::signal::Settings&) noexcept;
    static bool validScene(const RadioSceneFrameV1&,const RadioReceiverConfigV1&,bool semiActive) noexcept;
    static bool validIlluminationProgram(const radar::signal::Waveform&) noexcept;
    static double durationS(const radar::signal::Waveform&) noexcept;
    static double energySnr(const RadioReceiverConfigV1&,const RadioEchoSceneV1&,
        double receiverGainLinear,double frequencyHz,double dwellS,bool semiActive) noexcept;
    bool configure(const RadioReceiverConfigV1&,std::string&);
    void reset() noexcept;
    void observeIlluminationV1(const RadioSceneFrameV1&);
    RadioReceiverResultV1 expose(const RadioSceneFrameV1&,weapon::V3 pointing,
        const std::array<weapon::V3,3>& bodyAxes,std::uint64_t seed,bool semiActive);
    const RadioReceiverResultV1* latest() const noexcept {return cached_.valid?&cached_:nullptr;}
    double beamwidthRad(bool vertical=false) const noexcept {return pattern_.beamwidthRad(vertical);}
private:
    RadioReceiverConfigV1 config_;
    rf::Pattern pattern_;
    double patternFrequencyHz_=0,lastTimeS_=-1;
    radar::signal::Stream stream_;
    bool seeded_=false;
    RadioReceiverResultV1 cached_;
    RadioTemporalCoincidenceV1 temporal_;
    std::optional<radar::signal::Waveform> illuminationProgram_;
    std::optional<radar::IlluminationClockV1> illuminationClock_;
    double firstIlluminationS_=-1,lastIlluminationS_=-1;
};
} // namespace fsim::ms::components
