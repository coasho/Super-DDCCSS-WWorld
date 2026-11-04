#pragma once

#include "simulation/mission_systems/components/interfaces/ThermalSeekerExecutionV1.h"
#include <random>
#include <map>

namespace fsim::ms::components {

struct FrozenThermalImageFeatureV1 {
    weapon::V3 direction;
    double signalToNoise=0, angularVarianceRad2=0;
    ImageExtentV2 extent;
    double signedContrastElectrons=0;
    double centroidColumn=0,centroidRow=0; ///< Native bottom-row-zero detector coordinates.
    std::optional<std::size_t> refereeAttributionIndex;
};
struct FrozenThermalImageResultV1 {
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
    std::vector<FrozenThermalImageFeatureV1> features;
};

/// Native bounded image formation and detector. The measured image, not source
/// bearing/support, supplies the detection, error, shape and association input.
class FrozenThermalImageReceiverV1 {
public:
    static bool validConfig(const ThermalImageConfigV1&) noexcept;
    static bool validScene(const ThermalSceneFrameV1&,const ThermalImageConfigV1&) noexcept;
    bool configure(const ThermalImageConfigV1&,std::string&);
    void reset() noexcept;
    const FrozenThermalImageResultV1* latest() const noexcept {return cached_.valid?&cached_:nullptr;}
    FrozenThermalImageResultV1 expose(const ThermalSceneFrameV1&,weapon::V3 pointing,
        std::uint64_t seed,bool imaging);
    /// Planck band radiance of a grey surface: epsilon*sigma*T^4*f_band/pi.
    static double bandRadiance(double temperatureK,double emissivity,double lowUm,double highUm) noexcept;
private:
    friend class ThermalDualBandReceiverV1;
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
    FrozenThermalImageResultV1 cached_;
    double normal();
};

} // namespace fsim::ms::components
