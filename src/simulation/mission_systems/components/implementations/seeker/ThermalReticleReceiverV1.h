#pragma once
#include "ThermalImageReceiverV1.h"
#include "simulation/mission_systems/components/interfaces/ThermalReticleSeekerExecutionV1.h"
#include <random>

namespace fsim::ms::components {
struct ThermalReticleResultV1 {
    bool valid=false,complete=false,bearingResolved=false,saturated=false;
    double timeS=0,exposureStartTimeS=0,exposureEndTimeS=0,measurementTimeS=0;
    double fitSnr=0,fitResidual=0,varianceBoundElectrons2=0;
    double fullWellElectrons=0,readNoiseElectrons=0,darkElectrons=0,adcStepElectrons=0;
    std::vector<double> phases,incidentElectrons,expectedElectrons,measuredElectrons;
    std::vector<ThermalImageFeatureV1> features;
};
/// Photon-domain stationary spoke mask with nutating optics, one scalar
/// detector and scan-synchronous measured-waveform fitting. Optical grid cells
/// are numerical quadrature, never independently read detector pixels.
class ThermalReticleReceiverV1 {
public:
    static bool validConfig(const ThermalReticleConfigV1&) noexcept;
    bool configure(const ThermalReticleConfigV1&,std::string&);
    void reset() noexcept;
    ThermalReticleResultV1 expose(const ThermalSceneFrameV1&,weapon::V3 pointing,std::uint64_t seed);
    const ThermalReticleResultV1* latest() const noexcept {return cached_.valid?&cached_:nullptr;}
    std::optional<std::array<double,3>> captureWindowV1(double readTimeS) const noexcept;
    static double maskV1(const ThermalReticleConfigV1&,double x,double y,double phase) noexcept;
    static double angularSpeedV1(double x,double y,double radius,double phase,double frequency) noexcept;
    /// Internal inverse-optics calibration, not a detector exposure. The
    /// retained continuous renderer remains the independent fallback/oracle.
    static std::optional<std::vector<double>> unitGroundFieldV1(const ThermalImageConfigV1&,
        weapon::V3 pointing,weapon::V3 up,double cutoff,
        const std::optional<std::array<double,4>>& sourceRegion=std::nullopt,bool* accelerated=nullptr);
private:
    ThermalReticleConfigV1 config_;
    std::vector<double> weights_,timeWeights_,solidAngles_,flat_,templates_;
    std::vector<std::vector<double>> nuisance_;
    double startedS_=-1,lastTimeS_=-1,lastDetectorSampleTimeS_=-1,detectorCharge_=0,expectedCharge_=0;
    std::mt19937_64 rng_;
    bool seeded_=false;
    ThermalReticleResultV1 cached_;
    double normal();
    std::vector<double> spot(double x,double y,double sigma,double width=0,double height=0,bool removeNuisance=true) const;
    void project(std::vector<double>&) const;
    void fit(ThermalReticleResultV1&,weapon::V3 pointing,weapon::V3 up) const;
    void fitHorizon(ThermalReticleResultV1&,weapon::V3 pointing,weapon::V3 up) const;
};
}
