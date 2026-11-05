// Native referee diagnostics. This file serializes existing authoritative state;
// it never updates physics, guidance, sensor processing or actor knowledge.
#include "World.h"
#include "simulation/mission_systems/components/interfaces/ThermalSeekerExecutionV1.h"
#include "simulation/mission_systems/components/interfaces/ThermalDualBandSeekerExecutionV1.h"
#include "simulation/mission_systems/components/interfaces/RadioSeekerExecutionV1.h"
#include "simulation/mission_systems/components/implementations/seeker/ThermalImageReceiverV1.h"
#include "foundation/core/Log.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace fsim::session {
namespace {
using namespace ipc;
using Status=WeaponDiagnosticStatusV1;
template<std::size_t N> void text(char (&out)[N],std::string_view value) {
    weaponDiagnosticTextV1(out,value.substr(0,N-1));
}
template<class V> void vector(double (&out)[3],const V& v){out[0]=v.x;out[1]=v.y;out[2]=v.z;}
void stamp(WeaponDiagnosticStampV1& out,const WeaponDiagnosticFrameV1& frame,double sample,Status status,std::string_view cause) {
    out={};out.status=status;out.sourceFrame=frame.worldFrame;out.sourceId=frame.entity;
    out.sampleTimeS=sample;out.processingTimeS=sample;out.deliveredTimeS=frame.simTimeS;
    out.validUntilS=frame.simTimeS;text(out.cause,cause);
}
}
void World::recordWeaponEventGenerationV1(const ms::EngagementEvent& event,std::uint64_t source,std::uint64_t target) {
    if(!options_.publish||!event.serial)return;
    weaponEventGenerationsV1_[event.serial]={source,target};
    while(weaponEventGenerationsV1_.size()>8192)weaponEventGenerationsV1_.erase(weaponEventGenerationsV1_.begin());
}
void World::publishWeaponDiagnosticsV1() {
    if(!publisher_||!publisher_->active())return;
    std::uint64_t pid=0,frame=0;std::uint32_t incarnation=0;
    if(!ipc::weaponDiagnosticWorldIdentityV1(options_.name,pid,incarnation,frame)||!frame)return;
    if(!weaponDiagnosticsV1_) {
        auto sidecar=std::make_unique<ipc::WeaponDiagnosticsV1>();
        if(!sidecar->create(options_.name,pid,incarnation))return;
        weaponDiagnosticsV1_=std::move(sidecar);
        if(weaponDiagnosticCaptureBindingReadyV1_) {
            weaponDiagnosticsRecorderV1_=std::make_unique<ipc::WeaponDiagnosticsRecorderV1>();
            if(!weaponDiagnosticsRecorderV1_->open(options_.recordPath.string()+".wdiag",pid,incarnation))weaponDiagnosticsRecorderV1_.reset();
        }
    }
    if(!weaponThermalBandV1_) {
        auto selector=std::make_unique<ipc::WeaponThermalBandV1>();
        if(selector->create(options_.name,pid,incarnation))weaponThermalBandV1_=std::move(selector);
    }
    // Bounded retained displays survive source removal, preserving their
    // original generation and sensor epoch. No slot is rebound by index.
    for(auto it=weaponDiagnosticHistoryV1_.begin();it!=weaponDiagnosticHistoryV1_.end();) {
        if(!entry(it->first)&&simTime_>it->second->simTimeS+30)it=weaponDiagnosticHistoryV1_.erase(it);else ++it;
    }
    ipc::WeaponDiagnosticSelectionV1 request;const bool requested=weaponDiagnosticsV1_->selection(frame,request);
    Entry* selected=nullptr;
    if(requested&&request.entity)selected=entry(request.entity);
    if(selected&&(selected->info.role!=ms::Role::Weapon||selected->info.generation!=request.generation))selected=nullptr;
    if(!requested)for(auto& e:entries_)if(e&&e->info.role==ms::Role::Weapon&&(!selected||e->info.id>selected->info.id))selected=e.get();
    auto out=std::make_unique<ipc::WeaponDiagnosticFrameV1>();
    if(selected) {
        const auto retained=weaponDiagnosticHistoryV1_.find(selected->info.id);
        if(retained!=weaponDiagnosticHistoryV1_.end()&&retained->second->generation==selected->info.generation)*out=*retained->second;
        out->entity=selected->info.id;out->generation=selected->info.generation;
        text(out->name,selected->info.name);text(out->store,selected->origin.store);
        out->selectionStatus=Status::Available;
    } else if(requested&&request.entity) {
        const auto retained=weaponDiagnosticHistoryV1_.find(request.entity);
        if(retained!=weaponDiagnosticHistoryV1_.end()&&retained->second->generation==request.generation)*out=*retained->second;
        out->entity=request.entity;out->generation=request.generation;out->selectionStatus=Status::Removed;
    }
    out->publisherPid=pid;out->publisherIncarnation=incarnation;out->worldFrame=frame;out->simTimeS=simTime_;
    out->selectionRequest=requested?request.request:0;out->rosterCount=out->totalWeaponCount=out->eventCount=0;
    if(weaponThermalBandV1_) {
        unsigned bands=0;
        if(selected&&selected->seekerProvider) {
            const auto* dual=dynamic_cast<const ms::components::ThermalDualBandSeekerExecutionV1*>(selected->seekerProvider.get());
            const auto* single=dynamic_cast<const ms::components::ThermalSeekerExecutionV1*>(selected->seekerProvider.get());
            if(dual&&dual->thermalDualBandPolicyV1())bands=2;
            else if(single&&single->thermalPolicyV1())bands=1;
        }
        weaponThermalBandV1_->publish(frame,out->entity,out->generation,bands);
    }
    std::set<std::pair<std::uint32_t,std::uint64_t>> roster;
    const auto addRoster=[&](std::uint32_t id,std::uint64_t generation,std::string_view name,std::string_view store,bool retained) {
        if(!roster.emplace(id,generation).second)return;
        ++out->totalWeaponCount;if(out->rosterCount>=ipc::kWeaponDiagnosticRosterV1)return;
        auto& row=out->roster[out->rosterCount++];row={};row.entity=id;row.generation=generation;row.flags=retained?1u:0u;text(row.name,name);text(row.store,store);
    };
    for(const auto& e:entries_)if(e&&e->info.role==ms::Role::Weapon)addRoster(e->info.id,e->info.generation,e->info.name,e->origin.store,false);
    for(const auto& item:weaponDiagnosticHistoryV1_)addRoster(item.first,item.second->generation,item.second->name,item.second->store,true);
    pruneWeaponEffectsDiagnosticsV1();
    for(const auto& item:weaponEffectsDiagnosticsV1_)
        addRoster(item.first.first,item.first.second,item.second.name,item.second.store,true);
    if(selected&&!selected->weaponEnded)fillWeaponDiagnosticsV1(*selected,*out);
    else if(out->entity) {
        // Both a terminal entry and a retained source with no entry are
        // historical. Neither can authorize access to a live component/FDM.
        // A terminal receiver may still retain both actual last exposures.
        // Display selection must not resample it or rebuild a missing image.
        if(const auto* dual=selected?dynamic_cast<const ms::components::ThermalDualBandSeekerExecutionV1*>(selected->seekerProvider.get()):nullptr;dual&&dual->thermalDualBandV1())
            fillWeaponSensorDiagnosticsV1(*selected,*out);
        out->selectionStatus=Status::Removed;
        ipc::WeaponDiagnosticStampV1* ended[]={&out->thermal.stamp,&out->rf.stamp,&out->navigation.stamp,&out->control.stamp,&out->propulsion.stamp,&out->guidance.stamp,&out->guidance.measurementStamp,&out->link.stamp};
        for(auto* component:ended)if(component->status==Status::Available)component->status=Status::Removed;
    }
    fillWeaponEffectsDiagnosticsV1(*out);
    if(out->entity) {
        const auto belongs=[&](const ms::EngagementEvent& event) {
            const auto generation=weaponEventGenerationsV1_.find(event.serial);
            if(generation==weaponEventGenerationsV1_.end())return false;
            return (event.entity==out->entity&&generation->second.source==out->generation)||
                (event.target==out->entity&&generation->second.target==out->generation);
        };
        std::size_t first=combatEvents_.size();std::size_t count=0;
        while(first&&count<ipc::kWeaponDiagnosticEventsV1){--first;if(belongs(combatEvents_[first]))++count;}
        for(std::size_t i=first;i<combatEvents_.size()&&out->eventCount<ipc::kWeaponDiagnosticEventsV1;++i) {
            const auto& e=combatEvents_[i];if(!belongs(e))continue;
            const auto generation=weaponEventGenerationsV1_.at(e.serial);
            auto& row=out->events[out->eventCount++];row={};row.serial=e.serial;row.timeS=e.timeS;row.source=e.entity;row.target=e.target;
            row.sourceGeneration=generation.source;row.targetGeneration=generation.target;
            if(e.entity&&generation.source)row.flags|=ipc::WeaponDiagnosticEventSourceGenerationV1;
            if(e.target&&generation.target)row.flags|=ipc::WeaponDiagnosticEventTargetGenerationV1;
            text(row.cause,e.kind+": "+e.reason);
        }
    }
    const char* reason=nullptr;
    if(!ipc::validateWeaponDiagnosticsV1(*out,&reason)) {
        if(weaponDiagnosticFailureV1_!=(reason?reason:"unknown")){weaponDiagnosticFailureV1_=reason?reason:"unknown";LOG_WARN("session")<<"weapon diagnostics refused: "<<weaponDiagnosticFailureV1_;}
        return;
    }
    weaponDiagnosticFailureV1_.clear();
    if(!weaponDiagnosticsV1_->publish(*out))return;
    if(weaponDiagnosticsRecorderV1_&&!weaponDiagnosticsRecorderV1_->append(*out)) {
        LOG_WARN("session")<<"weapon diagnostics recording failed";weaponDiagnosticsRecorderV1_.reset();
    }
    if(selected) {
        if(!weaponDiagnosticHistoryV1_.count(out->entity)&&weaponDiagnosticHistoryV1_.size()>=8) {
            const auto oldest=std::min_element(weaponDiagnosticHistoryV1_.begin(),weaponDiagnosticHistoryV1_.end(),[](const auto& a,const auto& b){return a.second->simTimeS<b.second->simTimeS;});
            weaponDiagnosticHistoryV1_.erase(oldest);
        }
        weaponDiagnosticHistoryV1_[out->entity]=std::move(out);
    }
}
void World::fillWeaponDiagnosticsV1(const Entry& e,ipc::WeaponDiagnosticFrameV1& out) {
    using namespace ipc;
    auto& model=pool_->vehicle(e.slot);const auto& s=pool_->states()[e.slot];
    // A property inventory prevents optional absent diagnostics from creating
    // JSBSim properties or issuing warnings on every display frame.
    if(!e.diagnosticPropertiesBoundV1) {
        for(const char* prefix:{"fsim","propulsion","forces","external_reactions"})
            for(const auto& value:model.properties(prefix)) {
                const auto key=std::string(prefix)+"/"+value.first;
                e.diagnosticPropertiesV1.emplace(key,model.property(key));
            }
        e.diagnosticPropertiesBoundV1=true;
    }
    const auto property=[&](const std::string& key,double& value) {
        const auto p=e.diagnosticPropertiesV1.find(key);
        if(p==e.diagnosticPropertiesV1.end()||!p->second.valid())return false;
        const double native=p->second.get();if(!std::isfinite(native))return false;value=native;return true;
    };
    const auto binding=weaponLinksV1_.find(e.info.id);
    if(binding!=weaponLinksV1_.end()&&binding->second.strapdownNavigation) {
        const auto& nav=*binding->second.strapdownNavigation;const auto estimate=nav.estimate();auto& n=out.navigation;n={};
        stamp(n.stamp,out,estimate.timeS,estimate.valid?Status::Available:Status::Rejected,"Native body-IMU strapdown; no target truth corrections");
        n.errorStateCount=21;vector(n.positionEcefM,estimate.position);vector(n.velocityEcefMs,estimate.velocity);
        const auto q=nav.attitudeBodyToEcef();std::copy(q.begin(),q.end(),n.quaternionBodyToEcef);
        vector(n.accelerometerBiasBodyMs2,nav.accelerometerBiasBodyMs2());vector(n.gyroBiasBodyRadS,nav.gyroBiasBodyRadS());
        const auto imu=nav.imuSample();vector(n.measuredGyroBodyRadS,imu.gyroRadS);vector(n.measuredSpecificForceBodyMs2,imu.specificForceMs2);
        n.lastImuTimeS=estimate.lastImuSampleS;
        if(std::isfinite(estimate.lastGnssSampleS)){n.lastGnssTimeS=estimate.lastGnssSampleS;n.flags|=WeaponDiagnosticGnssEverSampledV1;}
        if(estimate.gnssAided)n.flags|=WeaponDiagnosticGnssAidedV1;
        if(e.origin.store=="aim-120d")n.flags|=WeaponDiagnosticGnssFittedV1;
        const auto& covariance=nav.errorCovariance();std::copy(covariance.begin(),covariance.end(),n.errorCovariance);
    } else stamp(out.navigation.stamp,out,simTime_,Status::Unavailable,"No V2 strapdown state installed for this release");
    fillWeaponSensorDiagnosticsV1(e,out);
    auto& guidance=out.guidance;guidance={};
    const auto& observation=e.seekerResultV2;
    const bool decision=e.guidance&&std::isfinite(e.guidanceDecisionTimeS)&&e.guidanceDecisionTimeS<=simTime_;
    stamp(guidance.stamp,out,decision?e.guidanceDecisionTimeS:simTime_,decision?Status::Available:Status::Unavailable,"Actual native guidance decision");
    guidance.phase=static_cast<std::uint32_t>(e.origin.phase);guidance.weaponAgeS=std::max(0.,simTime_-e.info.launchTimeS);
    if(observation.active)guidance.flags|=WeaponDiagnosticSeekerActiveV1;
    if(e.origin.tracking)guidance.flags|=WeaponDiagnosticGuidanceTrackingV1;
    if(e.origin.supported)guidance.flags|=WeaponDiagnosticSupportedV1;
    if(observation.midcourseEstimate)guidance.flags|=WeaponDiagnosticMidcourseV1;
    const bool delivered=observation.measurementTimeS&&observation.processingReadyTimeS&&observation.deliveredTimeS&&*observation.deliveredTimeS<=simTime_;
    stamp(guidance.measurementStamp,out,observation.measurementTimeS.value_or(simTime_),delivered?Status::Available:Status::Pending,"Original seeker exposure, processing and delivery epochs");
    if(delivered) {
        guidance.measurementStamp.processingTimeS=*observation.processingReadyTimeS;
        guidance.measurementStamp.deliveredTimeS=*observation.deliveredTimeS;
    }
    if(observation.track) {
        const auto& track=*observation.track;guidance.localTrackId=track.localId;guidance.trackTimeS=track.sampleTimeS;
        vector(guidance.losDirectionEcef,track.direction);guidance.flags|=WeaponDiagnosticLosV1;
        if(track.rateAvailable){vector(guidance.losDirectionRatePerS,track.directionRate);guidance.flags|=WeaponDiagnosticLosRateV1;}
        if(track.rangeM){guidance.rangeM=*track.rangeM;guidance.flags|=WeaponDiagnosticMeasuredRangeV1;}
        if(track.rangeRateMs){guidance.rangeRateMs=*track.rangeRateMs;guidance.flags|=WeaponDiagnosticMeasuredRangeRateV1;}
    }
    if(observation.angularVarianceRad2){guidance.angularVarianceRad2=*observation.angularVarianceRad2;guidance.flags|=WeaponDiagnosticAngularVarianceV1;}
    if(std::isfinite(e.origin.timeToGoS)&&e.origin.timeToGoS>=0){guidance.timeToGoS=e.origin.timeToGoS;guidance.flags|=WeaponDiagnosticTimeToGoV1;}
    if(std::isfinite(e.origin.timeToActiveS)&&e.origin.timeToActiveS>=0){guidance.timeToActiveS=e.origin.timeToActiveS;guidance.flags|=WeaponDiagnosticTimeToActiveV1;}
    auto& link=out.link;link={};
    if(binding!=weaponLinksV1_.end()&&binding->second.channel) {
        const auto& channel=*binding->second.channel;const auto& profile=channel.profile();const auto& stats=channel.stats();
        stamp(link.stamp,out,simTime_,Status::Available,"Actual finite modeled weapon radio; accepted delivered fusion only");
        link.direction=static_cast<std::uint32_t>(profile.direction);text(link.directionSource,profile.directionSource);
        link.flags=WeaponDiagnosticChannelStatsV1;
        if(entry(binding->second.owner)&&binding->second.generation==e.info.generation)link.flags|=WeaponDiagnosticChannelOpenV1;
        link.updatePeriodS=profile.updatePeriodS;link.feedbackPeriodS=profile.feedbackPeriodS;link.latencyS=profile.latencyS;
        link.dataRateBps=profile.dataRateBps;link.rangeM=profile.rangeM;link.frequencyHz=profile.frequencyHz;
        link.sent=stats.sent;link.delivered=stats.delivered;link.lost=stats.lost;link.masked=stats.masked;link.outOfRange=stats.outOfRange;
        link.queueRefused=stats.queueRefused;link.expired=stats.expired;link.rejected=stats.rejected;link.bytes=stats.bytes;
        if(const auto resource=weaponLinkResourcesV1_.find(binding->second.owner);resource!=weaponLinkResourcesV1_.end()) {
            link.resourceBusyUntilS=resource->second.busyUntilS;link.flags|=WeaponDiagnosticRadioResourceV1;
        }
        if(const auto input=channel.input()) {
            link.flags|=WeaponDiagnosticInputKnownV1;
            if(input->state!=ms::FusionStateV1::Lost&&simTime_<=input->validUntilS)link.flags|=WeaponDiagnosticInputValidV1;
            link.fusedEntity=input->id;link.fusionState=static_cast<std::uint32_t>(input->state);
            link.stateTimeS=input->timeS;link.lastUpdateS=input->lastUpdateS;link.inputReceivedS=channel.inputReceivedS();link.inputValidUntilS=input->validUntilS;
            std::copy_n(input->stateEcef,6,link.stateEcef);std::copy_n(input->covarianceEcef,36,link.covarianceEcef);
            if(input->sources.size()>64)link.stamp.status=Status::CapacityExceeded;
            else for(const auto& source:input->sources) {
                auto& row=link.sources[link.sourceCount++];row.kind=static_cast<std::uint32_t>(source.kind);row.system=source.system;row.payload=source.payload;row.track=source.track;
                row.sampleTimeS=source.sampleTimeS;row.receivedTimeS=source.receivedTimeS;row.flags=source.diagonalLinkCovariance?1u:0u;
            }
        }
    } else stamp(link.stamp,out,simTime_,Status::NotApplicable,"No modeled weapon radio endpoint");

    auto& control=out.control;control={};
    if(const auto native=weaponControlsV1(e.info.id);native&&native->available) {
        stamp(control.stamp,out,native->sampleTimeS,Status::Available,"Actual native finite fin plant");
        if(native->controllerAvailable){control.flags|=WeaponDiagnosticControllerV1;control.controllerTimeS=native->controllerTimeS;}
        if(native->controllerAvailable&&std::isfinite(e.finNavigationTimeS)) {
            control.navigationTimeS=e.finNavigationTimeS;control.flags|=WeaponDiagnosticControlNavigationEpochV1;
        }
        for(std::size_t k=0;k<3;++k)control.commands[k]=native->commands[k];
        if(native->controllerAvailable) {
            std::copy(e.finDemandBodyMs2.begin(),e.finDemandBodyMs2.end(),control.demandedAccelerationBodyMs2);
            control.flags|=WeaponDiagnosticAccelerationDemandV1;
        }
        const double mass=model.energy().massKg;
        const char* forceAxes[]={"forces/fbx-total-lbs","forces/fby-total-lbs","forces/fbz-total-lbs"};
        bool acceleration=std::isfinite(mass)&&mass>0;
        for(std::size_t k=0;k<3;++k) {
            double force=0;if(!property(forceAxes[k],force)){acceleration=false;break;}
            control.achievedAccelerationBodyMs2[k]=force*4.4482216152605/mass;
        }
        if(acceleration)control.flags|=WeaponDiagnosticAccelerationAchievedV1;
        for(const auto& fin:native->fins) {
            if(control.finCount==kWeaponDiagnosticFinsV1){control.stamp.status=Status::CapacityExceeded;break;}
            auto& row=control.fins[control.finCount++];row.id=fin.id;row.index=fin.nativeIndex;
            row.flags=(fin.valid?1u:0u)|(fin.failedStuck?2u:0u)|(fin.failedZero?4u:0u)|(fin.failedHardover?8u:0u);row.demandRad=std::isfinite(fin.demandRad)?fin.demandRad:0;row.actualRad=std::isfinite(fin.actualRad)?fin.actualRad:0;
            row.minimumRad=std::isfinite(fin.minimumRad)?fin.minimumRad:0;row.maximumRad=std::isfinite(fin.maximumRad)?fin.maximumRad:0;row.rateRadS=std::isfinite(fin.rateRadS)?fin.rateRadS:0;
        }
        double version=0;
        if(property("fsim/thrust-vector-control/version",version)&&version==1) {
            const bool present=property("fsim/thrust-vector-control/limit_rad",control.tvcLimitRad)&&property("fsim/thrust-vector-control/rate_rad_s",control.tvcRateRadS)&&
                property("fsim/thrust-vector-control/pitch-demand-rad",control.tvcDemandPitchRad)&&property("fsim/thrust-vector-control/yaw-demand-rad",control.tvcDemandYawRad)&&
                property("propulsion/engine/pitch-angle-rad",control.tvcActualPitchRad)&&property("propulsion/engine/yaw-angle-rad",control.tvcActualYawRad);
            if(present)control.flags|=WeaponDiagnosticTvcV1;
        }
    } else stamp(control.stamp,out,simTime_,Status::NotApplicable,"No live finite fin control snapshot");
    auto& propulsion=out.propulsion;propulsion={};
    stamp(propulsion.stamp,out,simTime_,Status::Available,"Actual JSBSim propulsion and fuel state");
    propulsion.engineCount=static_cast<std::uint32_t>(std::clamp(s.engineCount,0,4));
    propulsion.fuelKg=s.fuelKg;propulsion.flags|=WeaponDiagnosticFuelV1|WeaponDiagnosticFuelFlowV1;
    for(std::uint32_t k=0;k<propulsion.engineCount;++k) {
        propulsion.thrustN[k]=s.thrustN[k];
        propulsion.fuelFlowKgS[k]=model.engineFuelFlowKgS(static_cast<int>(k));
        if(s.thrustN[k]>0)propulsion.flags|=WeaponDiagnosticBurningV1;
    }
    if(property("fsim/motor-pulse/native-age-s",propulsion.nativeMotorAgeS))propulsion.flags|=WeaponDiagnosticMotorClockV1;
    if(e.plantAdapter.configured)propulsion.propulsionKind=e.plantAdapter.implementations[3];
    if(propulsion.propulsionKind==4) {
        propulsion.flags|=WeaponDiagnosticPulsePolicyV1;
        if(e.pulseManagerV1)propulsion.pulseState=static_cast<std::uint32_t>(e.pulseManagerV1->cachedDecisionV1().state);
    }
    double axial=0;
    if(property("fsim/ducted-inlet-v2/net-propulsion-force-x-n",axial)) {
        propulsion.netThrustN=axial;propulsion.flags|=WeaponDiagnosticNetAxialForceV1;
    } else if(propulsion.propulsionKind!=5&&propulsion.propulsionKind!=2&&property("forces/fbx-prop-lbs",axial)) {
        propulsion.netThrustN=axial*4.4482216152605;propulsion.flags|=WeaponDiagnosticNetAxialForceV1;
    }
    double command=0;if(property("fsim/motor-pulse/second-command",command)&&command>0)propulsion.flags|=WeaponDiagnosticSecondCommandV1;
    double ignition=0;if(property("fsim/motor-pulse/second-ignition-age-s",ignition)&&ignition>=0){propulsion.secondIgnitionAgeS=ignition;propulsion.flags|=WeaponDiagnosticSecondIgnitedV1;}
    const bool valve=property("fsim/ducted-rocket/valve-demand",propulsion.valveDemand)&&property("fsim/ducted-rocket/valve",propulsion.valveActual);
    if(valve)propulsion.flags|=WeaponDiagnosticValveV1;
    const bool inlet=property("fsim/ducted-rocket/engine[1]/air-kg-s",propulsion.inletAirKgS)&&property("fsim/ducted-rocket/engine[1]/inlet-temperature-k",propulsion.inletTemperatureK)&&
        property("fsim/ducted-rocket/engine[1]/total-temperature-k",propulsion.combustorTemperatureK);
    bool physicalInlet=inlet;
    if(propulsion.propulsionKind==5) {
        physicalInlet=property("fsim/ducted-inlet-v2/outflow-kg-s",propulsion.inletAirKgS)&&
            property("fsim/ducted-inlet-v2/engine[1]/total-temperature-k",propulsion.combustorTemperatureK);
        double enthalpy=0,cp=0;
        if(physicalInlet&&propulsion.inletAirKgS>0&&property("fsim/ducted-inlet-v2/outflow-enthalpy-w",enthalpy)&&
            property("fsim/ducted-inlet-v2/input-air_cp_j_kg_k",cp)&&cp>0)propulsion.inletTemperatureK=enthalpy/(propulsion.inletAirKgS*cp);
        else physicalInlet=false;
    }
    if(physicalInlet) {
        propulsion.flags|=WeaponDiagnosticInletV1;double burningFuel=0;
        const auto prefix=propulsion.propulsionKind==5?"fsim/ducted-inlet-v2/":"fsim/ducted-rocket/";
        if(property(std::string(prefix)+"engine[1]/burned-fuel-kg-s",burningFuel)&&burningFuel>0&&propulsion.inletAirKgS>0)
            propulsion.flags|=WeaponDiagnosticInletLitV1;
    }
    if(e.fuzeAdapter) {
        const auto& spec=e.fuzeAdapter->spec();auto& effects=out.effects;
        stamp(effects.stamp,out,simTime_,Status::Available,"Native fuze, warhead and causal fragment state");
        effects.flags|=WeaponDiagnosticFuzeV1;effects.fuzeAlgorithm=spec.fuzeAlgorithm;
        effects.armingTimeS=spec.armingS;effects.fuzeRadiusM=spec.fuzeM;
        if(simTime_-e.info.launchTimeS>=spec.armingS&&!e.origin.jettisoned)effects.flags|=WeaponDiagnosticArmedV1;
        if(e.warheadAdapter) {const auto& warhead=e.warheadAdapter->model();effects.flags|=WeaponDiagnosticWarheadV1;effects.warheadTopology=static_cast<std::uint32_t>(warhead.fragmentSource);effects.explosiveKg=warhead.explosiveKg;effects.caseMassKg=warhead.caseKg;}
    }
}
}
