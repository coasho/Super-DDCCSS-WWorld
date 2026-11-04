#pragma once
#include "ThermalImageReceiverV1.h"
#include "simulation/mission_systems/components/interfaces/ThermalDualBandSeekerExecutionV1.h"
#include <utility>

namespace fsim::ms::components {
struct ThermalDualBandFeatureV1 : ThermalImageFeatureV1 {
    std::optional<double> logSpectralRatio;
    double logSpectralVariance=0;
};
struct ThermalDualBandResultV1 {
    bool valid=false,spectralAssociationAvailable=false;
    double timeS=0;
    std::array<ThermalImageResultV1,2> channels;
    std::vector<ThermalDualBandFeatureV1> features;
};
class ThermalDualBandReceiverV1 {
public:
    static bool validConfig(const ThermalDualBandConfigV1&) noexcept;
    static bool validScene(const ThermalDualBandSceneFrameV1&,const ThermalDualBandConfigV1&) noexcept;
    bool configure(const ThermalDualBandConfigV1&,std::string&);
    void reset() noexcept;
    ThermalDualBandResultV1 expose(const ThermalDualBandSceneFrameV1&,weapon::V3,std::uint64_t);
    /// Commit only an actually selected measured feature from the latest
    /// exposure. The index is detector-local, never referee provenance.
    /// Same epoch/index retries succeed; changed or stale requests fail.
    bool commitSelectedMeasurementV1(double epoch,std::size_t featureIndex);
    const ThermalDualBandResultV1* latest() const noexcept {return cached_.valid?&cached_:nullptr;}
private:
    ThermalDualBandConfigV1 config_;
    std::array<ThermalImageReceiverV1,2> channels_;
    std::optional<ThermalDualBandFeatureV1> retained_;
    double retainedTimeS_=-1;
    std::optional<std::pair<double,std::size_t>> committedMeasurement_;
    ThermalDualBandResultV1 cached_;
};
}
