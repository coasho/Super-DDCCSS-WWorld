#pragma once
#include "simulation/mission_systems/components/support/MeasuredSeekerV2.h"
#include "ThermalImageReceiverV1.h"
#include "ThermalDualBandReceiverV1.h"
#include "RadioReceiverV1.h"
#include "ThermalReticleReceiverV1.h"
#include "simulation/mission_systems/components/support/ReceiverConfigurationV3.h"
#include "simulation/mission_systems/components/support/ReceiverConfigurationV1.h"
#include "simulation/mission_systems/components/interfaces/SeekerSceneClockV1.h"
#include "simulation/mission_systems/components/interfaces/SeekerLaunchNavigationV1.h"
#include <deque>
#include <cmath>
#include <memory>
#include <random>

namespace fsim::ms::components {
/// Finite detector/servo/measurement pipeline; V2 owns only family association.
class EngineeringSeekerV3 final : public SeekerProviderV1, public SeekerExecutionV2, public ThermalSeekerExecutionV1, public RadioSeekerExecutionV1, public ThermalDualBandSeekerExecutionV1, public SeekerSceneClockV1, public SeekerLaunchNavigationV1, public ThermalReticleSeekerExecutionV1 {
public:
    EngineeringSeekerV3(weapon::Spec,std::unique_ptr<MeasuredSeekerV2>);
    const weapon::SeekerConfigV1& config() const noexcept override;
    double infraredRange(double,double,bool) const noexcept override;
    double radarRange(double,double,bool) const noexcept override;
    bool notched(double,double) const noexcept override;
    double decoySignal(double,bool) const noexcept override;
    bool illuminated(const double[3],double,double,double,double) const noexcept override;
    SeekerResultV2 sample(const SeekerFrameV2&) override;
    bool configureThermalV1(const ThermalImageConfigV1&,std::string&) override;
    SeekerResultV2 sampleThermalV1(const SeekerFrameV2&,const ThermalSceneFrameV1&) override;
    const ThermalImageResultV1* thermalImageV1() const noexcept override {return thermal_.latest();}
    const ThermalImageConfigV1* thermalPolicyV1() const noexcept override {return thermalConfig_?&*thermalConfig_:nullptr;}
    bool configureRadioV1(const RadioReceiverConfigV1&,std::string&) override;
    SeekerResultV2 sampleRadioV1(const SeekerFrameV2&,const RadioSceneFrameV1&) override;
    const RadioReceiverResultV1* radioSpectrumV1() const noexcept override {return radio_.latest();}
    const RadioReceiverConfigV1* radioPolicyV1() const noexcept override {return radioConfig_?&*radioConfig_:nullptr;}
    bool configureReceiverV1(const ReceiverConfigurationV1&,std::string&);
    bool configureReceiverV2(const ReceiverConfigurationV2&,std::string&);
    bool configureReceiverV3(const ReceiverConfigurationV3&,std::string&);
    bool configureThermalReticleV1(const ThermalReticleConfigV1&,std::string&) override;
    SeekerResultV2 sampleThermalReticleV1(const SeekerFrameV2&,const ThermalSceneFrameV1&) override;
    const ThermalReticleResultV1* thermalReticleV1() const noexcept override {return reticle_.latest();}
    const ReticleReceiverConfigurationV1* reticleReceiverConfigurationV1() const noexcept {return reticleReceiverConfig_?&*reticleReceiverConfig_:nullptr;}
    bool configureThermalDualBandV1(const ThermalDualBandConfigV1&,std::string&) override;
    SeekerResultV2 sampleThermalDualBandV1(const SeekerFrameV2&,const ThermalDualBandSceneFrameV1&) override;
    const ThermalDualBandResultV1* thermalDualBandV1() const noexcept override {return thermalDual_.latest();}
    const ThermalDualBandConfigV1* thermalDualBandPolicyV1() const noexcept override {return dualConfig_?&*dualConfig_:nullptr;}
    const ReceiverConfigurationV1* receiverConfigurationV1() const noexcept {return physicalConfig_?&*physicalConfig_:nullptr;}
    const ReceiverConfigurationV2* receiverConfigurationV2() const noexcept {return receiverConfigV2_?&*receiverConfigV2_:nullptr;}
    void seedTrack(const SeekerTrackV2&) override;
    bool seedLaunchNavigationV1(const weapon::Observation&) override;
    void resetTracking() noexcept override;
    bool sceneCaptureDueV1(double timeS) const noexcept override {
        return std::isfinite(timeS)&&timeS>=0&&timeS>lastTimeS_&&
            (!seeded_||timeS+1e-12>=nextSampleS_);
    }
    /// Independent deterministic method seams, also used by execution.
    static double rfSnr(const weapon::SeekerConfigV1&,const SeekerSceneV3&,bool semi,double exposureS) noexcept;
    static double irSnr(const weapon::SeekerConfigV1&,const SeekerSceneV3&,double strength) noexcept;
    static double bandFraction(double temperatureK,double lowUm,double highUm) noexcept;
    static double detectionProbability(double snr,double threshold,bool infrared) noexcept;
    static double sourceAspectCosineV3(weapon::V3 observerToSource,weapon::V3 sourceForward) noexcept;
    static bool measuredAssociationV1(weapon::V3 measuredDirection,const SeekerTrackV2&,double captureTimeS,double readTimeS,double gateRad) noexcept;
private:
    struct Packet {double deliverS;SeekerResultV2 result;std::optional<weapon::V3> quadrantCentering;};
    weapon::Spec spec_;
    std::unique_ptr<MeasuredSeekerV2> family_;
    std::deque<Packet> pending_;
    std::optional<SeekerTrackV2> filtered_;
    bool filteredQuadrantUnresolved_=false;
    std::optional<LauncherEstimateV2> radarNavigation_;
    std::optional<LauncherEstimateV2> radarNavigationSupportPrior_;
    double radarSupportEpochS_=-1;
    std::uint32_t radarNavigationTrack_=0;
    std::optional<weapon::V3> quadrantCentering_;
    struct QuadrantConstraint {weapon::V3 normal;double timeS;};
    std::deque<QuadrantConstraint> quadrantBounds_;
    SeekerResultV2 cached_;
    weapon::V3 pointing_;
    double lastTimeS_=-1,nextSampleS_=0,firstTimeS_=0;
    bool seeded_=false;
    std::optional<std::array<weapon::V3,3>> previousAxes_;
    std::mt19937_64 rng_;
    std::mt19937_64 backgroundRng_;
    ThermalImageReceiverV1 thermal_;
    ThermalDualBandReceiverV1 thermalDual_;
    ThermalReticleReceiverV1 reticle_;
    std::optional<ThermalReticleConfigV1> reticleConfig_;
    std::optional<ReticleReceiverConfigurationV1> reticleReceiverConfig_;
    struct OpticalPose {double timeS;SeekerNavigationPoseV1 navigation;std::array<weapon::V3,3> physicalAxes;weapon::V3 pointing,up;};
    std::deque<OpticalPose> opticalPoseHistory_;
    std::optional<OpticalPose> opticalPoseAt(double) const;
    RadioReceiverV1 radio_;
    std::optional<ThermalImageConfigV1> thermalConfig_;
    std::optional<RadioReceiverConfigV1> radioConfig_;
    std::optional<ReceiverConfigurationV1> physicalConfig_;
    std::optional<ThermalDualBandConfigV1> dualConfig_;
    std::optional<ReceiverConfigurationV2> receiverConfigV2_;
    std::optional<double> radioActiveSinceS_;
    SeekerResultV2 sampleImpl(const SeekerFrameV2&,const ThermalSceneFrameV1*,const RadioSceneFrameV1* =nullptr,const ThermalDualBandSceneFrameV1* =nullptr);
    double normal();
};
}
