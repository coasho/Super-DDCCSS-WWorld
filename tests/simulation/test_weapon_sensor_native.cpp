#include "simulation/runtime/world/World.h"
#include "simulation/mission_systems/components/interfaces/MeasuredGuidanceV2.h"
#include "simulation/mission_systems/components/interfaces/ThermalSeekerExecutionV1.h"
#include "simulation/mission_systems/components/interfaces/RadioSeekerExecutionV1.h"
#include "simulation/mission_systems/components/interfaces/SeekerSceneClockV1.h"
#include "simulation/mission_systems/components/interfaces/SeekerLaunchNavigationV1.h"
#include "simulation/mission_systems/components/implementations/seeker/ThermalDualBandReceiverV1.h"
#include "simulation/mission_systems/components/implementations/seeker/RadioReceiverV1.h"
#include "simulation/mission_systems/components/support/MeasuredSeekerV2.h"
#include "simulation/mission_systems/rf/Path.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <set>

namespace {
using namespace fsim;
using namespace fsim::ms::components;
struct Clock {
    unsigned instance=0;double now=0,age=0;
    SeekerResultV2 result;ms::weapon::V3 demand;
    bool decision=false;
};
struct Observation {
    unsigned instances=0,samples=0,decisions=0,tracking=0,rawScenes=0;
    unsigned dualScenes=0,dualImages=0;
    unsigned groundPathChecks=0;
    unsigned lastReceiver=0;
    std::set<unsigned> decisionReceivers;
    session::World* fixtureWorld=nullptr;
    std::uint32_t launcher=0;
    std::shared_ptr<sim::GroundProvider> ground;
    std::string lastThermal;
    std::string selected;
    std::vector<Clock> rows;
    bool countermeasure=false,flare=false,dispensed=false;
    std::uint32_t target=0;
    unsigned inventoryBefore=0,inventoryAfter=0;
    double releaseTime=0,dispenseTime=0;
    std::array<bool,2> channelProof{};
    std::vector<std::string> countermeasureRows;
    std::string countermeasureDiagnostic;
    std::string thermalDebugRows;
    unsigned countermeasureDiagnosticCount=0;
    double countermeasureMinimumAngle=std::numeric_limits<double>::infinity();
};
// Referee-only geometric selection of a counterfactual scene contribution.
// No identity is supplied to a receiver or measured association operation.
template<class Patch> bool actualCountermeasure(const Observation& o,const SeekerFrameV2& f,const Patch& patch,double range,double& age) {
    if(!o.dispensed||!o.fixtureWorld||f.ageS<1.5)return false;
    const auto weapons=o.fixtureWorld->weapons();if(weapons.size()!=1)return false;
    if(std::abs(f.timeS-o.fixtureWorld->simTime())>1e-9||
       std::abs(f.ageS-(f.timeS-weapons.front().launchTimeS))>1e-9)return false;
    const auto* weapon=o.fixtureWorld->vehicleState(weapons.front().id);if(!weapon)return false;
    const ms::weapon::V3 weaponPosition{weapon->positionEcef[0],weapon->positionEcef[1],weapon->positionEcef[2]};
    unsigned matches=0;
    for(const auto& decoy:o.fixtureWorld->decoys())if(decoy.owner==o.target&&decoy.flare==o.flare&&decoy.ageS>=.1) {
        // f.position is deliberately the onboard navigation estimate. Referee
        // source attribution compares physical scene geometry with the actual
        // native origin, without passing that truth to association/guidance.
        const ms::weapon::V3 relative{decoy.positionEcef[0]-weaponPosition.x,decoy.positionEcef[1]-weaponPosition.y,decoy.positionEcef[2]-weaponPosition.z};
        const auto distance=ms::components::measured::norm(relative);
        const auto ray=ms::components::measured::unit(relative);
        if(std::abs(distance-range)<.01&&ms::components::measured::norm(ms::components::measured::subtract(ray,patch.direction))<1e-8){++matches;age=decoy.ageS;}
    }
    REQUIRE(matches<=1);return matches==1;
}
double maximumDifference(const std::vector<double>& a,const std::vector<double>& b) {
    REQUIRE(a.size()==b.size());double result=0;for(std::size_t i=0;i<a.size();++i)result=std::max(result,std::abs(a[i]-b[i]));return result;
}
double maximumDifference(const std::vector<float>& a,const std::vector<float>& b) {
    REQUIRE(a.size()==b.size());double result=0;for(std::size_t i=0;i<a.size();++i)result=std::max(result,std::abs(double(a[i])-double(b[i])));return result;
}
void probeThermal(Observation& o,const SeekerFrameV2& f,const ThermalSceneFrameV1& scene,
    const ThermalImageResultV1* actual,ThermalImageReceiverV1& receiver,bool& configured,
    const ThermalImageConfigV1& policy,unsigned channel,std::uint64_t seed,bool imaging) {
    if(!o.countermeasure||o.channelProof[channel]||!actual||actual->timeS!=f.timeS)return;
    std::string error;if(!configured){REQUIRE(receiver.configure(policy,error));configured=true;}
    auto before=receiver;
    const auto full=receiver.expose(scene,actual->pointing,seed,imaging);REQUIRE(full.valid);
    const auto replayError=maximumDifference(full.expectedElectrons,actual->expectedElectrons);
    CHECK(replayError==0); // Exact history, policy and actual pointing, before clipping/noise.
    auto without=scene;unsigned matched=0;double age=0,range=0;
    std::optional<ThermalRadiancePatchV1> sourcePatch;
    for(std::size_t i=without.patches.size();i-->0;)if(actualCountermeasure(o,f,without.patches[i],without.patches[i].rangeM,age)) {
        range=without.patches[i].rangeM;sourcePatch=without.patches[i];++matched;
        without.patches.erase(without.patches.begin()+static_cast<std::ptrdiff_t>(i));
        without.refereeAttributionIndices.erase(without.refereeAttributionIndices.begin()+static_cast<std::ptrdiff_t>(i));
    }
    if(!matched)return;
    REQUIRE(matched==1);
    const auto control=before.expose(without,actual->pointing,seed,imaging);REQUIRE(control.valid);
    double positive=0;unsigned saturated=0;
    for(std::size_t i=0;i<full.expectedElectrons.size();++i) {
        positive+=std::max(0.,full.expectedElectrons[i]-control.expectedElectrons[i]);
        saturated+=full.expectedElectrons[i]>=policy.fullWellElectrons;
    }
    if(!(positive>0)) {
        const double angle=sourcePatch?std::acos(std::clamp(ms::components::measured::dot(
            ms::components::measured::unit(sourcePatch->direction),ms::components::measured::unit(actual->pointing)),-1.,1.)):0;
        if(sourcePatch&&(angle<o.countermeasureMinimumAngle||o.countermeasureDiagnosticCount<16)) {
            ++o.countermeasureDiagnosticCount;o.countermeasureMinimumAngle=std::min(o.countermeasureMinimumAngle,angle);
            using namespace ms::components::measured;
            const auto forward=unit(actual->pointing),right=unit(cross(forward,unit(scene.up))),up=unit(cross(right,forward));
            std::ostringstream detail;detail<<std::setprecision(12)<<" t="<<f.timeS<<" age="<<age<<" range="<<range
                <<" angle="<<angle<<" az="<<std::atan2(dot(sourcePatch->direction,right),dot(sourcePatch->direction,forward))
                <<" el="<<std::atan2(dot(sourcePatch->direction,up),dot(sourcePatch->direction,forward))
                <<" patch="<<sourcePatch->widthRad<<','<<sourcePatch->heightRad<<" radiance="<<sourcePatch->radianceWm2Sr
                <<" positive="<<positive;
            o.countermeasureDiagnostic+=detail.str();
        }
        return; // Preserve the failed coupling predicate and actual native scene.
    }
    std::ostringstream row;row<<std::setprecision(17)<<channel<<",thermal,"<<f.timeS<<','<<f.ageS<<','<<age<<','<<range<<','<<matched
        <<','<<replayError<<','<<positive<<','<<maximumDifference(full.expectedElectrons,control.expectedElectrons)<<','<<saturated
        <<','<<maximumDifference(full.measuredElectrons,control.measuredElectrons)<<",0,0,0,0,0,0,preclip_charge\n";
    o.countermeasureRows.push_back(row.str());o.channelProof[channel]=replayError==0;
}
void describeThermal(Observation& observation,const ThermalImageResultV1* image,const SeekerFrameV2& frame,const SeekerResultV2& result,const ThermalSceneFrameV1& scene) {
    std::ostringstream detail;detail<<"frame="<<frame.timeS<<" samples="<<frame.samples.size()<<" patches="<<scene.patches.size()<<" tracking="<<result.tracking;
    if(observation.countermeasure&&std::getenv("FSIM_SENSOR_NATIVE_DEBUG_CM")&&image&&image->timeS==frame.timeS) {
        for(std::size_t i=0;i<scene.patches.size();++i) {
            const auto& patch=scene.patches[i];const auto& p=image->pointing;
            std::ostringstream row;row<<std::setprecision(17)<<frame.timeS<<','<<observation.lastReceiver<<','<<frame.ageS<<','<<i
                <<','<<patch.rangeM<<','<<std::acos(std::clamp(ms::components::measured::dot(p,patch.direction),-1.,1.))
                <<','<<patch.direction.x<<','<<patch.direction.y<<','<<patch.direction.z
                <<','<<p.x<<','<<p.y<<','<<p.z<<','<<image->features.size();
            if(!image->features.empty()) {
                const auto& feature=image->features.front();row<<','<<feature.direction.x<<','<<feature.direction.y<<','<<feature.direction.z
                    <<','<<feature.signalToNoise<<','<<feature.angularVarianceRad2;
            } else row<<",,,,,";
            row<<','<<result.tracking;
            if(result.track)row<<','<<result.track->direction.x<<','<<result.track->direction.y<<','<<result.track->direction.z;
            else row<<",,,";
            for(const auto q:image->quadrantElectrons)row<<','<<q;
            row<<'\n';observation.thermalDebugRows+=row.str();
        }
    }
    if(image) {
        detail<<" image="<<image->timeS<<" features="<<image->features.size();
        if(!image->expectedElectrons.empty())detail<<" expected_max="<<*std::max_element(image->expectedElectrons.begin(),image->expectedElectrons.end());
        detail<<" quadrant_variance="<<image->quadrantVarianceElectrons2;
        for(const auto charge:image->quadrantElectrons)detail<<" quadrant="<<charge;
        for(const auto& feature:image->features) {
            detail<<" feature_snr="<<feature.signalToNoise<<" direction="<<feature.direction.x<<","<<feature.direction.y<<","<<feature.direction.z;
            if(feature.refereeAttributionIndex)detail<<" attribution="<<*feature.refereeAttributionIndex;
        }
    } else detail<<" no image";
    observation.lastThermal=detail.str();
}
ms::rf::Terminal inversePosition(ms::weapon::V3 v) {
    constexpr double a=6378137.,e2=6.69437999014e-3;
    const double p=std::hypot(v.x,v.y);double lat=std::atan2(v.z,p*(1-e2)),height=0;
    for(unsigned i=0;i<10;++i) {
        const double n=a/std::sqrt(1-e2*std::sin(lat)*std::sin(lat));
        height=p/std::cos(lat)-n;lat=std::atan2(v.z,p*(1-e2*n/(n+height)));
    }
    return {lat,std::atan2(v.y,v.x),height};
}
void checkGroundPropagation(Observation& observation,const RadioSceneFrameV1& scene) {
    if(observation.groundPathChecks>=64)return;
    REQUIRE(observation.fixtureWorld);const auto weapons=observation.fixtureWorld->weapons();REQUIRE_FALSE(weapons.empty());
    const auto* state=observation.fixtureWorld->vehicleState(weapons.back().id);REQUIRE(state);
    ms::rf::PathOptions options;options.ground=observation.ground.get();options.frequencyHz=scene.frequencyHz;
    const ms::rf::Terminal own{state->latitudeRad,state->longitudeRad,state->altitudeMslM};
    for(const auto& echo:scene.groundPatches) {
        if(observation.groundPathChecks>=64)break;
        const ms::weapon::V3 at{state->positionEcef[0]+echo.direction.x*echo.receiverRangeM,
            state->positionEcef[1]+echo.direction.y*echo.receiverRangeM,state->positionEcef[2]+echo.direction.z*echo.receiverRangeM};
        const auto endpoint=inversePosition(at);
        const auto path=ms::rf::computePath(own,endpoint,options);
        CHECK_FALSE(path.terrainMasked);
        CHECK(echo.receiverExcessLossDb==Catch::Approx(std::max(0.,path.totalDb-path.freeSpaceDb)).margin(1e-5));
        if(scene.illuminationAvailable) {
            const auto* transmitter=observation.fixtureWorld->vehicleState(observation.launcher);REQUIRE(transmitter);
            const auto illumination=ms::rf::computePath({transmitter->latitudeRad,transmitter->longitudeRad,transmitter->altitudeMslM},endpoint,options);
            CHECK_FALSE(illumination.terrainMasked);
            CHECK(echo.transmitterExcessLossDb==Catch::Approx(std::max(0.,illumination.totalDb-illumination.freeSpaceDb)).margin(1e-5));
        }
        ++observation.groundPathChecks;
    }
}
class Sensor final:public SeekerProviderV1,public SeekerExecutionV2,public ThermalSeekerExecutionV1,public RadioSeekerExecutionV1,public ThermalDualBandSeekerExecutionV1,public SeekerSceneClockV1,public SeekerLaunchNavigationV1 {
public:
    Sensor(std::unique_ptr<SeekerProviderV1> p,std::shared_ptr<Observation> o):inner_(std::move(p)),out_(std::move(o)),instance_(++out_->instances){}
    const ms::weapon::SeekerConfigV1& config() const noexcept override{return inner_->config();}
    bool sceneCaptureDueV1(double timeS) const noexcept override {
        const auto* clock=dynamic_cast<const SeekerSceneClockV1*>(inner_.get());
        return !clock||clock->sceneCaptureDueV1(timeS);
    }
    bool seedLaunchNavigationV1(const ms::weapon::Observation& estimate) override {
        auto* initialization=dynamic_cast<SeekerLaunchNavigationV1*>(inner_.get());
        return initialization&&initialization->seedLaunchNavigationV1(estimate);
    }
    double infraredRange(double a,double b,bool c) const noexcept override{return inner_->infraredRange(a,b,c);}
    double radarRange(double a,double b,bool c) const noexcept override{return inner_->radarRange(a,b,c);}
    bool notched(double a,double b) const noexcept override{return inner_->notched(a,b);}
    double decoySignal(double a,bool b) const noexcept override{return inner_->decoySignal(a,b);}
    bool illuminated(const double a[3],double b,double c,double d,double e) const noexcept override{return inner_->illuminated(a,b,c,d,e);}
    SeekerResultV2 sample(const SeekerFrameV2& f) override {
        out_->lastReceiver=instance_;
        ++out_->samples;REQUIRE(config().engineering.version==3);REQUIRE(f.bodyAxes);
        out_->rawScenes+=static_cast<unsigned>(std::count_if(f.samples.begin(),f.samples.end(),[](const auto& s){return s.scene.has_value();}));
        auto r=execution().sample(f);out_->rows.push_back({instance_,f.timeS,f.ageS,r,{},false});return r;
    }
    bool configureThermalV1(const ThermalImageConfigV1& c,std::string& error) override {
        auto* p=dynamic_cast<ThermalSeekerExecutionV1*>(inner_.get());return p&&p->configureThermalV1(c,error);
    }
    SeekerResultV2 sampleThermalV1(const SeekerFrameV2& f,const ThermalSceneFrameV1& scene) override {
        out_->lastReceiver=instance_;
        ++out_->samples;REQUIRE(f.bodyAxes);REQUIRE(scene.navigation);CHECK(scene.navigation->valid);
        out_->rawScenes+=static_cast<unsigned>(scene.patches.size());
        auto* p=dynamic_cast<ThermalSeekerExecutionV1*>(inner_.get());REQUIRE(p);
        const auto r=p->sampleThermalV1(f,scene);describeThermal(*out_,p->thermalImageV1(),f,r,scene);
        if(const auto* policy=p->thermalPolicyV1())probeThermal(*out_,f,scene,p->thermalImageV1(),thermalProbes_[0],thermalConfigured_[0],*policy,0,f.receiverSeed,config().mode==ms::weapon::SeekerModeV1::ImagingInfrared);
        out_->rows.push_back({instance_,f.timeS,f.ageS,r,{},false});return r;
    }
    const ThermalImageResultV1* thermalImageV1() const noexcept override {
        const auto* p=dynamic_cast<const ThermalSeekerExecutionV1*>(inner_.get());return p?p->thermalImageV1():nullptr;
    }
    const ThermalImageConfigV1* thermalPolicyV1() const noexcept override {
        const auto* p=dynamic_cast<const ThermalSeekerExecutionV1*>(inner_.get());return p?p->thermalPolicyV1():nullptr;
    }
    bool configureThermalDualBandV1(const ThermalDualBandConfigV1& c,std::string& error) override {
        auto* p=dynamic_cast<ThermalDualBandSeekerExecutionV1*>(inner_.get());return p&&p->configureThermalDualBandV1(c,error);
    }
    SeekerResultV2 sampleThermalDualBandV1(const SeekerFrameV2& f,const ThermalDualBandSceneFrameV1& scene) override {
        out_->lastReceiver=instance_;
        ++out_->samples;++out_->dualScenes;REQUIRE(f.bodyAxes);
        for(const auto& channel:scene.channels) {
            REQUIRE(channel.navigation);CHECK(channel.navigation->valid);CHECK(channel.timeS==f.timeS);
            out_->rawScenes+=static_cast<unsigned>(channel.patches.size());
        }
        auto* p=dynamic_cast<ThermalDualBandSeekerExecutionV1*>(inner_.get());REQUIRE(p);
        const auto r=p->sampleThermalDualBandV1(f,scene);
        if(const auto* image=p->thermalDualBandV1())if(const auto* policy=p->thermalDualBandPolicyV1())
            for(unsigned channel=0;channel<2;++channel)probeThermal(*out_,f,scene.channels[channel],&image->channels[channel],thermalProbes_[channel],thermalConfigured_[channel],policy->channels[channel],channel,
                f.receiverSeed^(channel?0x42414e4430303032ull:0x42414e4430303031ull),true);
        if(const auto* measured=p->thermalDualBandV1();measured&&measured->valid) {
            REQUIRE(measured->channels[0].valid);REQUIRE(measured->channels[1].valid);
            CHECK(measured->channels[0].timeS==measured->channels[1].timeS);
            CHECK_FALSE(measured->channels[0].measuredElectrons.empty());CHECK_FALSE(measured->channels[1].measuredElectrons.empty());
            ++out_->dualImages;
        }
        out_->rows.push_back({instance_,f.timeS,f.ageS,r,{},false});return r;
    }
    const ThermalDualBandResultV1* thermalDualBandV1() const noexcept override {
        const auto* p=dynamic_cast<const ThermalDualBandSeekerExecutionV1*>(inner_.get());return p?p->thermalDualBandV1():nullptr;
    }
    const ThermalDualBandConfigV1* thermalDualBandPolicyV1() const noexcept override {
        const auto* p=dynamic_cast<const ThermalDualBandSeekerExecutionV1*>(inner_.get());return p?p->thermalDualBandPolicyV1():nullptr;
    }
    bool configureRadioV1(const RadioReceiverConfigV1& c,std::string& error) override {
        auto* p=dynamic_cast<RadioSeekerExecutionV1*>(inner_.get());return p&&p->configureRadioV1(c,error);
    }
    SeekerResultV2 sampleRadioV1(const SeekerFrameV2& f,const RadioSceneFrameV1& scene) override {
        out_->lastReceiver=instance_;
        ++out_->samples;REQUIRE(f.bodyAxes);REQUIRE(scene.navigation);CHECK(scene.navigation->valid);
        out_->rawScenes+=static_cast<unsigned>(scene.echoes.size()+scene.groundPatches.size());
        checkGroundPropagation(*out_,scene);
        auto* p=dynamic_cast<RadioSeekerExecutionV1*>(inner_.get());REQUIRE(p);
        const auto r=p->sampleRadioV1(f,scene);
        probeRadio(f,scene,*p);
        out_->rows.push_back({instance_,f.timeS,f.ageS,r,{},false});return r;
    }
    const RadioReceiverResultV1* radioSpectrumV1() const noexcept override {
        const auto* p=dynamic_cast<const RadioSeekerExecutionV1*>(inner_.get());return p?p->radioSpectrumV1():nullptr;
    }
    const RadioReceiverConfigV1* radioPolicyV1() const noexcept override {
        const auto* p=dynamic_cast<const RadioSeekerExecutionV1*>(inner_.get());return p?p->radioPolicyV1():nullptr;
    }
    void seedTrack(const SeekerTrackV2& t) override{execution().seedTrack(t);}
    void resetTracking() noexcept override{if(auto* p=dynamic_cast<SeekerExecutionV2*>(inner_.get()))p->resetTracking();}
private:
    void probeRadio(const SeekerFrameV2& f,const RadioSceneFrameV1& scene,const RadioSeekerExecutionV1& execution) {
        if(!out_->countermeasure||out_->channelProof[0])return;
        const auto* policy=execution.radioPolicyV1();REQUIRE(policy);std::string error;
        if(!radioConfigured_){REQUIRE(radioProbe_.configure(*policy,error));radioConfigured_=true;}
        const bool semi=config().mode==ms::weapon::SeekerModeV1::SemiActiveRadar;
        if(semi)radioProbe_.observeIlluminationV1(scene);
        const auto* actual=execution.radioSpectrumV1();if(!actual||actual->timeS!=f.timeS)return;
        auto before=radioProbe_;
        const auto full=radioProbe_.expose(scene,actual->pointing,*f.bodyAxes,f.receiverSeed,semi);REQUIRE(full.valid);
        REQUIRE(full.spectrum.power.size()==actual->spectrum.power.size());
        const double replayError=maximumDifference(full.spectrum.power,actual->spectrum.power);CHECK(replayError==0);
        auto without=scene;unsigned matched=0;double age=0,range=0;std::size_t index=0;
        for(std::size_t i=0;i<without.echoes.size();++i)if(actualCountermeasure(*out_,f,without.echoes[i],without.echoes[i].receiverRangeM,age)) {
            index=i;range=without.echoes[i].receiverRangeM;++matched;
        }
        if(!matched||!full.spectrum.kept)return;
        REQUIRE(matched==1);
        // Keep the same echo count/order and strictly positive native energy:
        // deleting an echo changes phase/fluctuation RNG consumption. A 1e-200
        // RCS control removes its float-cell contribution while retaining draws.
        // The retained first interval precedes peak-dependent later RNG draws.
        const auto& source=scene.echoes[index];without.echoes[index].rcsM2*=1e-200;
        const auto control=before.expose(without,actual->pointing,*f.bodyAxes,f.receiverSeed,semi);REQUIRE(control.valid);
        if(!control.spectrum.kept)return;
        const auto difference=maximumDifference(full.spectrum.power,control.spectrum.power);
        using namespace ms::components::measured;
        const auto forward=unit(actual->pointing);auto right=unit(subtract((*f.bodyAxes)[1],scale(forward,dot((*f.bodyAxes)[1],forward))));
        if(norm(right)<.5)right=unit(cross((*f.bodyAxes)[2],forward));
        const auto down=unit(cross(forward,right));
        const double face[3]={dot(source.direction,policy->aperture.electronic?(*f.bodyAxes)[0]:forward),
            dot(source.direction,policy->aperture.electronic?(*f.bodyAxes)[1]:right),dot(source.direction,policy->aperture.electronic?(*f.bodyAxes)[2]:down)};
        const ms::rf::Pattern pattern(policy->aperture,scene.frequencyHz);
        const double gain=pattern.gain(face,policy->aperture.electronic?dot(forward,(*f.bodyAxes)[1]):0,policy->aperture.electronic?dot(forward,(*f.bodyAxes)[2]):0);
        const auto energy=RadioReceiverV1::energySnr(*policy,source,gain,scene.frequencyHz,full.integratedTimeS,semi);
        const auto residual=RadioReceiverV1::energySnr(*policy,without.echoes[index],gain,scene.frequencyHz,full.integratedTimeS,semi);
        REQUIRE(std::isfinite(energy));REQUIRE(energy>0);REQUIRE(residual>0);CHECK(residual/energy==Catch::Approx(1e-200));
        // A changed retained cell is direct receiver coupling, not a claim of
        // detection, deception, target association or equipment effectiveness.
        if(!(difference>0))return;
        std::ostringstream row;row<<std::setprecision(17)<<"0,radio,"<<f.timeS<<','<<f.ageS<<','<<age<<','<<range<<','<<matched
            <<','<<replayError<<",0,0,0,0,"<<difference<<','<<energy<<','<<residual<<','<<semi<<','<<bool(scene.illuminationWaveform)<<','<<bool(scene.illuminationClock)<<",retained_first_interval\n";
        out_->countermeasureRows.push_back(row.str());out_->channelProof[0]=replayError==0;
    }
    SeekerExecutionV2& execution(){auto* p=dynamic_cast<SeekerExecutionV2*>(inner_.get());REQUIRE(p);return *p;}
    std::unique_ptr<SeekerProviderV1> inner_;std::shared_ptr<Observation> out_;unsigned instance_;
    std::array<ThermalImageReceiverV1,2> thermalProbes_;
    std::array<bool,2> thermalConfigured_{};
    RadioReceiverV1 radioProbe_;bool radioConfigured_=false;
};
class Controller final:public GuidanceProviderV1,public MeasuredGuidanceV2 {
public:
    Controller(std::unique_ptr<GuidanceProviderV1> p,std::shared_ptr<Observation> o):inner_(std::move(p)),out_(std::move(o)){}
    ms::weapon::Spec spec() const noexcept override{return inner_->spec();}
    ms::weapon::Output update(const ms::weapon::Input& f) override{return inner_->update(f);}
    ms::weapon::Output updateMeasured(const SeekerFrameV2& f,const SeekerResultV2& r) override {
        out_->decisionReceivers.insert(out_->lastReceiver);
        ++out_->decisions;if(r.tracking)++out_->tracking;
        auto* p=dynamic_cast<MeasuredGuidanceV2*>(inner_.get());REQUIRE(p);const auto output=p->updateMeasured(f,r);
        for(const double value:{output.acceleration.x,output.acceleration.y,output.acceleration.z})CHECK(std::isfinite(value));
        if(r.measurementTimeS) {
            REQUIRE(r.deliveredTimeS);REQUIRE(r.processingReadyTimeS);
            CHECK(*r.measurementTimeS<=*r.processingReadyTimeS);
            CHECK(*r.processingReadyTimeS<=*r.deliveredTimeS+1e-9);
            CHECK(*r.deliveredTimeS<=f.timeS+1e-9);
            CHECK(*r.deliveredTimeS-*r.processingReadyTimeS<f.dtS+1e-9);
        }
        if(r.tracking) {REQUIRE(r.track);CHECK(r.covarianceAvailable);REQUIRE(r.angularVarianceRad2);CHECK(*r.angularVarianceRad2>0);}
        out_->rows.push_back({0,f.timeS,f.ageS,r,output.acceleration,true});return output;
    }
private:
    std::unique_ptr<GuidanceProviderV1> inner_;std::shared_ptr<Observation> out_;
};
ProviderRegistryV1 observe(const std::string& id,std::shared_ptr<Observation> out) {
    const std::string mechanism(equipment::mechanism(id));
    REQUIRE(mechanism.size()>3);REQUIRE(mechanism.substr(mechanism.size()-3)==".v3");out->selected=id;
    const auto defaults=builtInProvidersV1();auto registry=defaults;std::string error;
    const bool registered=registry.seeker.registerFactory(id,mechanism,[table=defaults.seeker,out,mechanism](const auto& d,const auto& context,auto& e)->std::unique_ptr<SeekerProviderV1>{
        REQUIRE(d.implementationId==mechanism);auto p=table.create(d,context,e);if(!p)return {};
        return std::make_unique<Sensor>(std::move(p),out);
    },true,error);
    REQUIRE(registered);
    REQUIRE(registry.guidance.registerFactory("fsim.guidance.measured_pn.v2","fsim.guidance.measured_pn.v2",
        [table=defaults.guidance,out](const auto& d,const auto& context,auto& e)->std::unique_ptr<GuidanceProviderV1>{
            auto p=table.create(d,context,e);if(!p)return {};return std::make_unique<Controller>(std::move(p),out);
        },true,error));
    return registry;
}
session::VehicleSpec carrier(const char* name,const char* model="f16c") {
    session::VehicleSpec s;s.name=name;s.type=std::string("jsbsim:")+model;
    s.initial.latitudeDeg=36.25;s.initial.longitudeDeg=-115.03;s.initial.altitudeMslM=6000;
    s.initial.airspeedTrueMs=250;s.initial.headingDeg=90;return s;
}
void optional(std::ostream& output,const std::optional<double>& value) {if(value&&std::isfinite(*value))output<<*value;}
void dump(const std::string& id,int skip,const Observation& data,const ms::WeaponRecord& weapon) {
    const auto* path=std::getenv("FSIM_SENSOR_NATIVE_DIR");if(!path)return;
    const auto folder=std::filesystem::path(path);std::filesystem::create_directories(folder);
    std::ofstream file(folder/(id+"-skip"+std::to_string(skip)+".csv"));REQUIRE(file.good());file<<std::setprecision(17);
    file<<"instance,decision,time_s,launch_age_s,tracking,measurement_s,processing_ready_s,delivered_s,angular_variance_rad2,point_x,point_y,point_z,demand_x,demand_y,demand_z\n";
    for(const auto& row:data.rows) {
        file<<row.instance<<','<<row.decision<<','<<row.now<<','<<row.age<<','<<row.result.tracking<<',';
        optional(file,row.result.measurementTimeS);file<<',';optional(file,row.result.processingReadyTimeS);file<<',';
        optional(file,row.result.deliveredTimeS);file<<',';optional(file,row.result.angularVarianceRad2);
        file<<','<<row.result.pointingDirection.x<<','<<row.result.pointingDirection.y<<','<<row.result.pointingDirection.z
            <<','<<row.demand.x<<','<<row.demand.y<<','<<row.demand.z<<'\n';
    }
    std::ofstream summary(folder/("summary-skip"+std::to_string(skip)+".csv"),std::ios::app);REQUIRE(summary.good());
    summary<<std::setprecision(17)<<id<<','<<data.selected<<','<<skip<<','<<data.instances<<','<<data.samples<<','<<data.decisions<<','<<data.tracking
        <<','<<ms::weaponEndName(weapon.end)<<',';
    if(std::isfinite(weapon.missM))summary<<weapon.missM;
    summary<<'\n';
}
void dumpCountermeasure(const std::string& id,const Observation& data,const ms::WeaponRecord& weapon) {
    const auto* path=std::getenv("FSIM_SENSOR_NATIVE_DIR");if(!path)return;
    const auto folder=std::filesystem::path(path);std::filesystem::create_directories(folder);
    std::ofstream diagnostic(folder/(id+"-countermeasure-diagnostic.txt"));
    diagnostic<<data.countermeasureDiagnostic<<'\n';
    if(!data.thermalDebugRows.empty()) {
        std::ofstream debug(folder/(id+"-thermal-debug.csv"));
        debug<<"time_s,instance,age_s,patch,range_m,angle_rad,ray_x,ray_y,ray_z,point_x,point_y,point_z,features,feature_x,feature_y,feature_z,snr,variance,tracking,track_x,track_y,track_z,q0,q1,q2,q3\n"
            <<data.thermalDebugRows;
    }
    std::ofstream file(folder/(id+"-countermeasure-skip4.csv"));REQUIRE(file.good());
    file<<"channel,receiver,time_s,launch_age_s,decoy_age_s,range_m,matched,full_replay_max_error,positive_expected_electrons,max_expected_delta,saturated_pixels,max_measured_delta,retained_cell_max_delta,native_energy_snr,residual_energy_snr,semi_active,illumination_waveform,illumination_clock,proof\n";
    for(const auto& row:data.countermeasureRows)file<<row;
    std::ofstream summary(folder/"countermeasure-summary-skip4.csv",std::ios::app);REQUIRE(summary.good());
    summary<<std::setprecision(17)<<id<<','<<data.selected<<",4,"<<(data.flare?"flare":"chaff")<<','<<data.inventoryBefore<<','<<data.inventoryAfter
        <<','<<data.releaseTime<<','<<data.dispenseTime<<','<<data.countermeasureRows.size()<<','<<data.channelProof[0]<<','<<data.channelProof[1]
        <<','<<data.samples<<','<<data.decisions<<','<<data.tracking<<','<<ms::weaponEndName(weapon.end)<<',';
    if(std::isfinite(weapon.missM))summary<<weapon.missM;
    summary<<'\n';
}
void population(int skip,std::string_view selected={},std::shared_ptr<sim::GroundProvider> ground={},bool countermeasure=false) {
    if(const auto* shard=std::getenv("FSIM_SENSOR_NATIVE_CARD")) {
        REQUIRE(selected.empty());REQUIRE(*shard!='\0');selected=shard;
    }
    if(const auto* path=std::getenv("FSIM_SENSOR_NATIVE_DIR")) {
        const auto folder=std::filesystem::path(path);std::filesystem::create_directories(folder);
        std::ofstream summary(folder/(countermeasure?"countermeasure-summary-skip4.csv":"summary-skip"+std::to_string(skip)+".csv"),std::ios::trunc);
        REQUIRE(summary.good()); // Every execution owns a fresh, complete population receipt.
    }
    std::vector<std::string> errors;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=ms::readStoreLibrary(library,errors);REQUIRE(errors.empty());unsigned count=0;
    for(const auto& [id,card]:cards)if(card->storeClass=="aam"&&(selected.empty()||id==selected)) {
        INFO(id);INFO(skip);++count;ProfileV1 profile;std::string error;
        REQUIRE(readProfileV1(library/(id+".json"),*card,profile,error));
        auto observations=std::make_shared<Observation>();session::WorldOptions options;
        observations->ground=ground?ground:std::make_shared<sim::FlatGround>(0);options.ground=observations->ground;
        options.name="sensor-native-"+id+"-"+std::to_string(skip);options.dt=1./120;options.frameSkip=skip;
        options.publish=false;options.workers=1;options.pinWorkers=false;options.seed=17;options.jsbsimRoot=FSIM_TEST_JSBSIM_ROOT;
        session::World world(options);observations->fixtureWorld=&world;
        REQUIRE(world.setPayloadProvidersV1(observe(profile.find("seeker")->implementationId,observations),error));
        const bool semi=card->seeker=="semi_active_radar";const bool heat=card->seeker.find("infrared")!=std::string::npos;
        observations->countermeasure=countermeasure;observations->flare=heat;
        auto ownSpec=carrier("launcher",semi?"fa18c":"f16c");const auto own=world.createVehicle(ownSpec);REQUIRE(own);observations->launcher=own;
        REQUIRE(world.command(own,control::VelocityCommand{250,0,control::kHold,control::kHold}));
        std::optional<std::uint16_t> station,radar;
        for(const auto& p:world.missionSystems(own)->payloads()) {
            if(p.family==ms::Family::StoreStation)station=p.index;
            if(p.family==ms::Family::AirRadar)radar=p.index;
        }
        REQUIRE(station);REQUIRE(radar);const auto* stationCard=world.missionSystems(own)->card(*station);REQUIRE(stationCard);
        int slot=0;const double mass=card->value("mass_kg",0)+card->value("fuel_kg",0);
        for(const auto& row:stationCard->stations)if(row.maxKg>=mass&&std::find(row.takes.begin(),row.takes.end(),"aam")!=row.takes.end()){slot=row.number;break;}
        REQUIRE(slot>0);REQUIRE(world.loadStore(own,*station,slot,id)==ms::Reason::None);
        for(const auto payload:{*station,*radar})REQUIRE(world.submit(own,ms::StateCommand{payload,ms::PayloadState::Operate}).accepted());
        for(unsigned step=0;step<unsigned(std::ceil(45/(options.dt*skip)));++step) {
            if(world.missionSystems(own)->payloads()[*station].state==ms::PayloadState::Operate&&world.missionSystems(own)->payloads()[*radar].state==ms::PayloadState::Operate)break;
            world.step();
        }
        REQUIRE(world.missionSystems(own)->payloads()[*station].state==ms::PayloadState::Operate);
        REQUIRE(world.missionSystems(own)->payloads()[*radar].state==ms::PayloadState::Operate);
        const auto* ownState=world.vehicleState(own);REQUIRE(ownState);
        auto targetSpec=carrier("target");targetSpec.initial.latitudeDeg=ownState->latitudeRad*180/std::acos(-1.);
        targetSpec.initial.longitudeDeg=ownState->longitudeRad*180/std::acos(-1.)+3000/(6371000*std::cos(ownState->latitudeRad))*180/std::acos(-1.);
        targetSpec.initial.altitudeMslM=6300;const auto target=world.createVehicle(targetSpec);REQUIRE(target);
        observations->target=target;
        REQUIRE(world.command(target,control::VelocityCommand{250,0,control::kHold,control::kHold}));
        std::optional<std::uint16_t> dispenser;
        if(countermeasure) {
            for(const auto& payload:world.missionSystems(target)->payloads())if(payload.family==ms::Family::Countermeasures)dispenser=payload.index;
            REQUIRE(dispenser);REQUIRE(world.submit(target,ms::StateCommand{*dispenser,ms::PayloadState::Operate}).accepted());
            for(unsigned step=0;step<unsigned(std::ceil(45/(options.dt*skip)))&&world.missionSystems(target)->payloads()[*dispenser].state!=ms::PayloadState::Operate;++step)world.step();
            REQUIRE(world.missionSystems(target)->payloads()[*dispenser].state==ms::PayloadState::Operate);
            const auto inventory=world.countermeasures(target);observations->inventoryBefore=heat?inventory.flare:inventory.chaff;REQUIRE(observations->inventoryBefore>0);
        }
        ms::AmtiCommand scan;scan.payload=*radar;scan.bars=1;scan.azimuthHalfWidthRad=.2617993877991494;
        scan.elevationCentreRad=.099668652491162;scan.rangeScaleM=60000;REQUIRE(world.submit(own,scan).accepted());
        std::uint64_t track=0;
        for(unsigned step=0;step<unsigned(std::ceil(20/(options.dt*skip)))&&!track;++step) {
            world.step();for(const auto& t:world.amtiTracks(own,*radar))if(t.updates>=3){track=t.id;break;}
        }
        REQUIRE(track);const auto designation=world.designate(own,*radar,track);REQUIRE(designation>=session::World::kDesignations);
        if(semi) {ms::AmtiCommand stt;stt.payload=*radar;stt.task=ms::AmtiTask::Track;stt.mode=ms::AmtiMode::SingleTargetTrack;
            stt.track=track;stt.illuminate=true;stt.rangeScaleM=60000;REQUIRE(world.submit(own,stt).accepted());world.step(3);}
        unsigned lockedReceiver=0;
        if(heat) {
            for(unsigned step=0;step<unsigned(std::ceil(10/(options.dt*skip)))&&world.seekerTarget(own,id)!=target;++step)world.step();
            INFO(observations->lastThermal);
            const auto* observedTarget=world.vehicleState(target);const auto* observedOwn=world.vehicleState(own);
            REQUIRE(observedTarget);REQUIRE(observedOwn);
            INFO("target alt="<<observedTarget->altitudeMslM<<" tas="<<observedTarget->airspeedTrueMs
                 <<" crashed="<<observedTarget->crashed<<" diverged="<<observedTarget->diverged
                 <<" own alt="<<observedOwn->altitudeMslM<<" pitch="<<observedOwn->eulerRad[1]);
            REQUIRE(world.seekerTarget(own,id)==target);
            lockedReceiver=observations->lastReceiver;REQUIRE(lockedReceiver>0);
        }
        ms::StrikeCommand strike;strike.payload=*station;strike.store=id;strike.target=designation;strike.consent=true;
        REQUIRE(world.submit(own,strike).accepted());
        for(unsigned step=0;step<unsigned(std::ceil(10/(options.dt*skip)))&&world.weapons().empty();++step)world.step();
        REQUIRE(world.weapons().size()==1);const auto weaponId=world.weapons().front().id;
        observations->releaseTime=world.simTime();
        const double end=world.simTime()+40;
        while(world.simTime()<end&&world.weapons().front().end==ms::WeaponEnd::InFlight) {
            if(countermeasure&&!observations->dispensed&&world.simTime()-observations->releaseTime>=1.5) {
                REQUIRE(world.dispense(target,heat)==ms::Reason::None);
                observations->dispensed=true;observations->dispenseTime=world.simTime();
            }
            world.step();
            if(countermeasure&&std::getenv("FSIM_SENSOR_NATIVE_DEBUG_CM")&&observations->dispensed&&
               world.simTime()-observations->dispenseTime>4.1&&!observations->channelProof[0]) {
                dumpCountermeasure(id,*observations,world.weapons().front());
                INFO(observations->countermeasureDiagnostic);
                FAIL("Bounded failed receiver diagnostic; not an acceptance run");
            }
        }
        const auto records=world.weapons();REQUIRE(records.size()==1);CHECK(records.front().id==weaponId);CHECK_FALSE(records.front().jettisoned);
        CHECK(observations->instances>0);CHECK(observations->rawScenes>0);CHECK(observations->decisions>1);CHECK(observations->tracking>1);
        if(heat) {
            INFO("On-rail receiver="<<lockedReceiver);
            CHECK(observations->decisionReceivers==std::set<unsigned>{lockedReceiver});
        }
        if(id=="mica-ir"){CHECK(observations->dualScenes>1);CHECK(observations->dualImages>1);}
        if(!heat)CHECK(observations->groundPathChecks>0);
        if(countermeasure) {
            INFO(observations->countermeasureDiagnostic);
            const auto inventory=world.countermeasures(target);observations->inventoryAfter=heat?inventory.flare:inventory.chaff;
            CHECK(observations->dispensed);CHECK(observations->inventoryBefore==observations->inventoryAfter+1);
            CHECK(observations->channelProof[0]);if(id=="mica-ir")CHECK(observations->channelProof[1]);
            dumpCountermeasure(id,*observations,records.front());
        } else dump(id,skip,*observations,records.front());
    }
    CHECK(count==(selected.empty()?24:1));
}
}
TEST_CASE("Every configured AAM executes native sensing at frameSkip1", "[weapon-sensor-native][fleet]"){population(1);}
TEST_CASE("Every configured AAM executes native sensing at frameSkip2", "[weapon-sensor-native][fleet]"){population(2);}
TEST_CASE("Every configured AAM executes native sensing at frameSkip4", "[weapon-sensor-native][fleet]"){population(4);}
TEST_CASE("Every configured AAM receives an actual native countermeasure in its physical detector", "[countermeasure-receiver-native][fleet]"){population(4,{}, {},true);}
TEST_CASE("AIM-9M retains actual on-rail acquisition in the frozen native scene", "[weapon-sensor-native][thermal-native-regression]"){population(4,"aim-9m");}
TEST_CASE("Native RF clutter observes the actual intervening terrain on its propagation path", "[weapon-sensor-native][rf-ground-propagation]") {
    class Ridge final:public sim::GroundProvider {
    public:
        double heightAboveEllipsoidM(double,double lon) const override {
            const double east=(lon+115.03*std::acos(-1.)/180)*6371000*std::cos(36.25*std::acos(-1.)/180);
            const double relative=(east-3000)/500;
            return 4500*std::exp(-relative*relative);
        }
    };
    for(const auto* model:{"aim-120c","aim-7m"})population(4,model,std::make_shared<Ridge>());
}
