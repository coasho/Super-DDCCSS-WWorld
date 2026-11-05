#pragma once
#include "ThermalImageReceiverV1.h"
#include "simulation/mission_systems/components/interfaces/ThermalReticleSeekerExecutionV1.h"
#include <random>
#include <functional>

namespace fsim::ms::components {
/// One synchronous unit-field work report. No retained observer state.
struct ThermalGroundWorkV2 {
    double totalMicros=0,originalMicros=0;
    unsigned fallbackReason=0,maxRefinementDepth=0;
    std::size_t originalCalls=0,activeColumns=0,activeRowVisits=0;
    std::size_t atEvaluations=0,ruleEvaluations=0,adaptiveCalls=0,repeatedAtSamples=0;
    ThermalOpaqueMemoWorkV2 memo;
};

struct ThermalReticleHorizonDiagnosticV1 {
    unsigned refusal=0; ///< 0 not attempted; see retained observation-code map.
    bool fitAvailable=false,budgetAvailable=false,informationAvailable=false,alternativeAvailable=false;
    std::array<double,5> shape{},alternativeShape{};
    std::array<double,3> rawBudget{}; ///< Sky, ground, source; no scene radiance input.
    weapon::V3 ownUp{};
    double nullCutoff=0,nullResidual=0,sourceResidual=0,localVariance=0,informationDeterminant=0,informationTrace=0;
    double alternativeDelta=0,alternativeSeparation=0;
};
struct ThermalReticleResultV1 {
    ThermalReticleHorizonDiagnosticV1 horizonDiagnostic;
    bool valid=false,complete=false,bearingResolved=false,saturated=false;
    bool preciseBearing=false; ///< Local angular information supports rate estimation.
    double fittedCentreXRad=0,fittedCentreYRad=0,fittedWidthRad=0,fittedHeightRad=0;
    double compactFitSnr=0,compactFitResidual=0;
    unsigned ambiguousAlternativeKind=0;
    double alternativeAmplitude=0,alternativeCostDifference=0,alternativeSky=0,alternativeSource=0;
    std::array<double,4> alternativeShape{};
    bool alternativeRadianceAvailable=false;
    double timeS=0,exposureStartTimeS=0,exposureEndTimeS=0,measurementTimeS=0;
    weapon::V3 pointing{}; ///< Exact optical-axis input used for this capture.
    double fitSnr=0,fitResidual=0,varianceBoundElectrons2=0;
    double fullWellElectrons=0,readNoiseElectrons=0,darkElectrons=0,adcStepElectrons=0;
    std::vector<double> phases,incidentElectrons,expectedElectrons,measuredElectrons;
    std::vector<ThermalImageFeatureV1> features;
};
/// Additive internal calibration output. No V1 layout or plugin vtable changes.
/// Rates are inferred from this measured exposure and are not guidance authority.
struct ThermalReticleMovingDiagnosticV2 {
    unsigned version=2,refusal=0,branch=0;
    bool attempted=false,fitAvailable=false,accepted=false,precise=false,domainBoundary=false;
    double readTimeS=0,captureTimeS=0,firstPhotonTimeS=0,lastPhotonTimeS=0;
    double firstStateTimeS=0,lastStateTimeS=0;
    double degrees=0,logHypotheses=0,evidenceThreshold=0,residualBound=0;
    double residual=0,nullResidual=0,angularVarianceRad2=0;
    std::array<double,7> endpointsWidthHeightCutoff{}; ///< State-epoch angles(4), fixed width/height, cutoff.
    std::array<double,3> rawBudget{};
    weapon::V3 captureDirection{},captureDirectionRate{};
    std::size_t basisEvaluations=0,regionalFields=0,calibrations=0;
};
/// Caller-owned measured-history proposal. Both vectors use the SAME Cartesian
/// frame as the current expose/infer pointing and up. direction is the accepted
/// bearing at captureTimeS, not extrapolated truth. No prior admits a detection.
struct ThermalReticleMeasuredPriorV2 {
    double captureTimeS=0,availableTimeS=0;
    weapon::V3 direction{},directionRate{};
    double widthRad=0,heightRad=0,cutoff=0; ///< Initial guesses, not transported support/uncertainty.
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
    /// Additive measured-track start; the pointer is synchronous and never retained.
    /// The old expose signature remains the exact no-prior path.
    ThermalReticleResultV1 exposeWithMeasuredPriorV2(const ThermalSceneFrameV1&,weapon::V3 pointing,
        std::uint64_t seed,const ThermalReticleMeasuredPriorV2*);
    void inferMeasuredWithPriorV2(ThermalReticleResultV1&,weapon::V3 pointing,weapon::V3 up,
        const ThermalReticleMeasuredPriorV2*) const;
    /// Pure five-parameter static-horizon calibration, excluding MovingV2.
    /// No detector exposure, cached receiver state, RNG or scene truth is used.
    void inferStaticHorizonForCalibrationV2(ThermalReticleResultV1&,weapon::V3 pointing,weapon::V3 up,
        const ThermalReticleMeasuredPriorV2* prior=nullptr) const;
    /// Internal inverse calibration shared by expose(): measured scalar charge,
    /// configured optics and own optical pose only. Does not mutate RNG/cache.
    void inferMeasuredV1(ThermalReticleResultV1&,weapon::V3 pointing,weapon::V3 up) const;
    /// Versioned, measured-only within-scan moving-radiator calibration.
    /// Optional output is synchronous/caller-owned; no forward pole/RNG access.
    void inferMovingMeasuredV2(ThermalReticleResultV1&,weapon::V3 pointing,weapon::V3 up,
        ThermalReticleMovingDiagnosticV2* diagnostic=nullptr) const;
    /// Pure instrument-basis calibration, not an exposure or a detection.
    std::optional<std::array<std::vector<double>,2>> movingBasisForCalibrationV2(
        const ThermalReticleResultV1& clock,weapon::V3 pointing,weapon::V3 up,
        const std::array<double,7>& endpointsWidthHeightCutoff,unsigned branch=0) const;
    /// Pure basis for production's version4 continuous regional numerical operator.
    std::optional<std::array<std::vector<double>,2>> movingBasisForCalibrationV4(
        const ThermalReticleResultV1& clock,weapon::V3 pointing,weapon::V3 up,
        const std::array<double,7>& endpointsWidthHeightCutoff,unsigned branch=0) const;
    /// Pure basis for version5 fast moments with the retained field error budget.
    std::optional<std::array<std::vector<double>,2>> movingBasisForCalibrationV5(
        const ThermalReticleResultV1& clock,weapon::V3 pointing,weapon::V3 up,
        const std::array<double,7>& endpointsWidthHeightCutoff,unsigned branch=0) const;
    const ThermalReticleResultV1* latest() const noexcept {return cached_.valid?&cached_:nullptr;}
    std::optional<std::array<double,3>> captureWindowV1(double readTimeS) const noexcept;
    static double maskV1(const ThermalReticleConfigV1&,double x,double y,double phase) noexcept;
    static double angularSpeedV1(double x,double y,double radius,double phase,double frequency) noexcept;
    /// Necessary raw sky/ground radiance domain for a null-hypothesis fit.
    static std::optional<double> physicalBackgroundAmplitudeV1(double skyAtZero,double skyProjection,double unconstrained) noexcept;
    /// Internal uncertainty calculus: centreX/Y, source width/height, local variance.
    static std::optional<double> finiteMirrorEnvelopeV1(const ThermalReticleConfigV1&,
        const std::vector<std::array<double,5>>& modes) noexcept;
    /// Internal inverse-optics calibration, not a detector exposure. The
    /// retained continuous renderer remains the independent fallback/oracle.
    static std::optional<std::vector<double>> unitGroundFieldV1(const ThermalImageConfigV1&,
        weapon::V3 pointing,weapon::V3 up,double cutoff,
        const std::optional<std::array<double,4>>& sourceRegion=std::nullopt,bool* accelerated=nullptr);
    /// V2 synchronous memo provider returns the exact original native unit
    /// background for this policy/pointing/up/cutoff; no authority is retained.
    static std::optional<std::vector<double>> unitGroundFieldMemoV2(const ThermalImageConfigV1&,
        weapon::V3 pointing,weapon::V3 up,double cutoff,
        const std::optional<std::array<double,4>>& sourceRegion,
        const std::function<const std::vector<double>*()>& nativeBackground,bool* accelerated=nullptr);
    static std::optional<std::vector<double>> unitGroundFieldProfileV2(const ThermalImageConfigV1&,
        weapon::V3 pointing,weapon::V3 up,double cutoff,
        const std::optional<std::array<double,4>>& sourceRegion,
        const std::function<const std::vector<double>*()>& nativeBackground,ThermalGroundWorkV2*,bool* accelerated=nullptr);
private:
    ThermalReticleConfigV1 config_;
    std::vector<double> weights_,timeWeights_,solidAngles_,flat_,templates_;
    std::vector<double> supportPrefix_;
    unsigned supportColumns_=0,supportRows_=0,supportMargin_=0;
    double supportPitch_=0;
    std::vector<std::vector<double>> nuisance_;
    double startedS_=-1,lastTimeS_=-1,lastDetectorSampleTimeS_=-1,detectorCharge_=0,expectedCharge_=0;
    std::mt19937_64 rng_;
    bool seeded_=false;
    ThermalReticleResultV1 cached_;
    double normal();
    std::vector<double> spot(double x,double y,double sigma,double width=0,double height=0,bool removeNuisance=true) const;
    std::array<std::vector<double>,4> spotJacobianV1(double x,double y,double width,double height) const;
    std::optional<std::array<double,2>> rawSkyFitV1(const ThermalReticleResultV1&,const std::vector<double>& rawTemplate) const;
    std::optional<double> physicalSourceAmplitudeV1(const ThermalReticleResultV1&,const std::array<double,4>&,double amplitude) const;
    std::optional<double> localAngularVarianceV1(const std::array<double,4>&,double amplitude,double variance,double lattice) const;
    void project(std::vector<double>&) const;
    void fit(ThermalReticleResultV1&,weapon::V3 pointing,weapon::V3 up) const;
    void fitHorizon(ThermalReticleResultV1&,weapon::V3 pointing,weapon::V3 up,bool* backgroundExplained=nullptr,bool nullOnly=false) const;
    void fitHorizonWithMeasuredPriorV2(ThermalReticleResultV1&,weapon::V3 pointing,weapon::V3 up,
        bool* backgroundExplained,bool nullOnly,const ThermalReticleMeasuredPriorV2*) const;
};
}
