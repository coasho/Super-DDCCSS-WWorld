#pragma once

#include "simulation/mission_systems/components/interfaces/ThermalSeekerExecutionV1.h"
#include <random>
#include <map>

namespace fsim::ms::components {

/// Synchronous optional optical-work counters, never simulation authority.
struct ThermalOpaqueMemoWorkV2 {
    std::size_t queries=0,hits=0,capacityMisses=0,maxNodes=0;
    std::size_t rootsBuilt=0,yHits=0,yEvaluations=0,probabilityHits=0,probabilityEvaluations=0;
};

/// Noiseless optical quadrature before any detector response, electron draw or
/// image/quadrant processing. This is a simulation integration grid, not an
/// additional imaging detector. Consumers must apply their physical aperture
/// and modulation before sampling their detector noise.
struct ThermalIncidentFieldV1 {
    std::vector<double> radiance;
    std::vector<std::vector<double>> sourceContrast;
    std::vector<std::size_t> sourceOrder;
    std::vector<double> backgroundRadiance;
};

struct ThermalImageFeatureV1 {
    weapon::V3 direction;
    double signalToNoise=0, angularVarianceRad2=0;
    ImageExtentV2 extent;
    double signedContrastElectrons=0;
    double centroidColumn=0,centroidRow=0; ///< Native bottom-row-zero detector coordinates.
    std::optional<std::size_t> refereeAttributionIndex;
};
struct ThermalImageResultV1 {
    bool valid=false;
    double timeS=0;
    unsigned width=0,height=0;
    weapon::V3 pointing; ///< Exact expose input; optics normalizes its own basis.
    ThermalImageConfigV1 config;
    double exposureStartTimeS=0;
    std::vector<double> expectedElectrons, measuredElectrons;
    bool imageProcessingAvailable=false,segmentationAvailable=false;
    std::vector<double> backgroundElectrons,contrastElectrons,noiseSigmaElectrons,detectionThresholdElectrons;
    /// Zero is no retained component, otherwise one-based measured feature
    /// index. No object/source identity is encoded in this detector mask.
    std::vector<std::uint16_t> segmentation;
    std::array<double,4> quadrantElectrons{}; ///< upper-left, upper-right, lower-left, lower-right signed contrast.
    double quadrantBackgroundElectrons=0,quadrantVarianceElectrons2=0;
    std::vector<ThermalImageFeatureV1> features;
};

/// Native bounded image formation and detector. The measured image, not source
/// bearing/support, supplies the detection, error, shape and association input.
class ThermalImageReceiverV1 {
public:
    static bool validConfig(const ThermalImageConfigV1&) noexcept;
    static bool validScene(const ThermalSceneFrameV1&,const ThermalImageConfigV1&) noexcept;
    bool configure(const ThermalImageConfigV1&,std::string&);
    void reset() noexcept;
    const ThermalImageResultV1* latest() const noexcept {return cached_.valid?&cached_:nullptr;}
    ThermalImageResultV1 expose(const ThermalSceneFrameV1&,weapon::V3 pointing,
        std::uint64_t seed,bool imaging);
    /// Planck band radiance of a grey surface: epsilon*sigma*T^4*f_band/pi.
    static double bandRadiance(double temperatureK,double emissivity,double lowUm,double highUm) noexcept;
    static std::optional<ThermalIncidentFieldV1> incidentFieldV1(
        const ThermalSceneFrameV1&,weapon::V3 pointing,const ThermalImageConfigV1&);
    static double gaussianIntervalV1(double low,double high,double detectorLow,double detectorHigh,double sigma);
private:
    friend class ThermalDualBandReceiverV1;
    friend class ThermalReticleReceiverV1;
    static std::optional<ThermalIncidentFieldV1> incidentFieldImplV1(
        const ThermalSceneFrameV1&,weapon::V3,const ThermalImageConfigV1&,const std::vector<double>*);
    static std::optional<ThermalIncidentFieldV1> incidentFieldProfileV2(
        const ThermalSceneFrameV1&,weapon::V3,const ThermalImageConfigV1&,const std::vector<double>*,ThermalOpaqueMemoWorkV2*);
    std::optional<std::size_t> apertureAttribution(double,double,double,double) const;
    ThermalImageConfigV1 config_;
    std::vector<double> pixelResponse_;
    // Referee-only current-frame charge and unknown historical response.
    // Frame-local slots are never treated as stable cross-frame identities.
    // Neither these keys nor values enter image measurements or association.
    std::map<std::size_t,std::vector<double>> attributionResponse_;
    std::vector<double> unattributedResponse_;
    std::mt19937_64 rng_;
    double lastTimeS_=-1;
    bool seeded_=false;
    ThermalImageResultV1 cached_;
    double normal();
};

} // namespace fsim::ms::components
