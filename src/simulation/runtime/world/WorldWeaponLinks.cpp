#include "World.h"
#include "simulation/mission_systems/rf/Path.h"
#include "simulation/mission_systems/weapon/TrackKinematics.h"
#include "simulation/runtime/services/AgentRules.h"
#include "simulation/mission_systems/weapon/NavigationConfigurationV2.h"
#include "simulation/flight/JsbsimModel.h"
#include <stdexcept>
#include <atomic>
#include <cmath>

namespace fsim::session {
namespace {
ms::weapon::NavigationStrapdownInputV2 nativeNavigationInputV2(const sim::VehicleState& state,double time,bool gnss) {
    using V=ms::weapon::V3;
    ms::weapon::NavigationStrapdownInputV2 in;in.timeS=time;in.gnssAvailable=gnss;
    in.position={state.positionEcef[0],state.positionEcef[1],state.positionEcef[2]};
    double velocity[3];ms::geom::velocityEcef(state,velocity);in.velocity={velocity[0],velocity[1],velocity[2]};
    const auto* c=state.rotationBodyToEcef;
    in.bodyToEcef={V{c[0],c[3],c[6]},V{c[1],c[4],c[7]},V{c[2],c[5],c[8]}};
    const auto* w=state.angularRateBodyRadS;const auto* v=state.velocityBodyMs;const auto* a=state.accelerationBodyMs2;
    in.angularRateBodyRelativeEcefRadS={w[0],w[1],w[2]};
    // GetUVWdot is the derivative of ECEF-relative velocity RESOLVED in
    // rotating body axes. Resolve du/dt + omega_body/ECEF x u to ECEF.
    const double body[3]={a[0]+w[1]*v[2]-w[2]*v[1],a[1]+w[2]*v[0]-w[0]*v[2],a[2]+w[0]*v[1]-w[1]*v[0]};
    in.acceleration={c[0]*body[0]+c[1]*body[1]+c[2]*body[2],
        c[3]*body[0]+c[4]*body[1]+c[5]*body[2],c[6]*body[0]+c[7]*body[1]+c[8]*body[2]};
    return in;
}
bool estimate(const ms::FusedEntityV1& entity,double now,sim::VehicleState& state) {
    if(entity.state==ms::FusionStateV1::Lost||now>entity.validUntilS||entity.timeS>now)return false;
    const auto dt=now-entity.timeS;
    for(int k=0;k<3;++k)state.positionEcef[k]=entity.stateEcef[2*k]+dt*entity.stateEcef[2*k+1];
    const double x=state.positionEcef[0],y=state.positionEcef[1],z=state.positionEcef[2];
    const double lon=std::atan2(y,x),p=std::hypot(x,y);double lat=std::atan2(z,p*(1-.0066943799901413165));
    double height=0;
    for(int k=0;k<8;++k){const auto n=6378137./std::sqrt(1-.0066943799901413165*std::sin(lat)*std::sin(lat));
        height=p/std::max(1e-12,std::cos(lat))-n;lat=std::atan2(z,p*(1-.0066943799901413165*n/(n+height)));}
    state.latitudeRad=lat;state.longitudeRad=lon;state.altitudeMslM=height;
    const double sl=std::sin(lat),cl=std::cos(lat),so=std::sin(lon),co=std::cos(lon);
    const double vx=entity.stateEcef[1],vy=entity.stateEcef[3],vz=entity.stateEcef[5];
    state.velocityNedMs[0]=-sl*co*vx-sl*so*vy+cl*vz;
    state.velocityNedMs[1]=-so*vx+co*vy;state.velocityNedMs[2]=-cl*co*vx-cl*so*vy-sl*vz;
    state.eulerRad[2]=std::atan2(state.velocityNedMs[1],state.velocityNedMs[0]);
    state.eulerRad[1]=std::atan2(-state.velocityNedMs[2],std::hypot(state.velocityNedMs[0],state.velocityNedMs[1]));
    state.airspeedTrueMs=std::hypot(std::hypot(vx,vy),vz);
    return ms::weapon::completeTrackKinematics(state);
}
}
std::shared_ptr<WeaponLinkAccessV1> World::weaponLinkAccessV1() {
    if(!weaponLinkAccess_) {
        static std::atomic<std::uint64_t> next{1};
        weaponLinkAccess_=std::make_shared<WeaponLinkAccessV1>();weaponLinkAccess_->world=this;
        if(!weaponLinkInstance_)weaponLinkInstance_=next.fetch_add(1);
    }
    return weaponLinkAccess_;
}
std::optional<ms::FusedEntityV1> World::fusedWeaponInputV1(std::uint32_t owner,std::uint32_t target) const {
    const auto* d=designation(target);if(!d||d->owner!=owner||d->retiredV1)return std::nullopt;
    for(const auto& entity:fusedTracksV1(owner)) {
        if(d->fusedEntityV1&&entity.id==d->fusedEntityV1)return entity;
        if(!d->fusedEntityV1)for(const auto& source:entity.sources)
            if(source.payload==d->payload&&source.track==d->track&&
               ((d->source==comm::kNoSystem&&source.kind==ms::FusionSourceKindV1::LocalRadar)||
                (d->source!=comm::kNoSystem&&source.kind==ms::FusionSourceKindV1::DeliveredLink&&source.system==d->source)))return entity;
    }
    return std::nullopt;
}
std::uint32_t World::designateFusedV1(std::uint32_t owner,std::uint64_t instance,std::uint64_t entityId) {
    if(instance!=weaponLinkInstance_||!entry(owner)||designations_.size()>=4096)return 0;
    for(const auto& entity:fusedTracksV1(owner))if(entity.id==entityId) {
        sim::VehicleState state{};if(!estimate(entity,simTime_,state))return 0;
        Designation d;d.id=kDesignations+static_cast<std::uint32_t>(designations_.size())+1;
        d.owner=owner;d.fusedEntityV1=entityId;d.valid=true;d.estimate=state;designations_.push_back(d);return d.id;
    }
    return 0;
}
bool World::resolveFusedWeaponTargetV1(std::uint32_t owner,std::uint32_t target,sim::VehicleState& out)const {
    const auto input=fusedWeaponInputV1(owner,target);return input&&estimate(*input,simTime_,out);
}
std::vector<ms::FusedDesignationV1> World::fusedDesignationsV1(std::uint32_t owner)const {
    std::vector<ms::FusedDesignationV1> out;
    for(const auto& d:designations_)if(d.owner==owner&&d.fusedEntityV1)out.push_back({d.id,d.fusedEntityV1,d.valid});
    return out;
}
void World::bindWeaponLinkV1(Entry& weapon,const Entry& launcher,const ms::StoreCard& card,const ms::weapon::Spec& spec) {
    weaponLinkAccessV1();
    ms::WeaponLinkProfileV1 profile;
    profile.directionSource="No modeled radio fitted/energized for this released store; no return capability inferred";
    const bool energized=weapon.guidance&&!weapon.origin.jettisoned;
    if(energized&&spec.launcherUpdates) {
        profile.direction=ms::WeaponLinkDirectionV1::Uplink;
        profile.directionSource="Engineering receive fit: authored guidance_capabilities.launcher_updates; return capability unknown";
    }
    // Exact variants and integrations only. Sources establish direction, not
    // proprietary payload fields, frequencies, rates or fitted performance.
    const auto& type=launcher.info.type;
    if(energized&&card.id=="aim-120d"&&type=="jsbsim:f16c") {
        profile.direction=ms::WeaponLinkDirectionV1::TwoWay;
        profile.directionSource="AIM-120D direction: DoD FY2026 Weapons.pdf; F-16 integration: AFMC 2012-02-13 Edwards Red Flag (F-16 M6.1+); payload/timing are engineering assumptions";
    }
    if(energized&&card.id=="meteor"&&type=="jsbsim:gripen") {
        profile.direction=ms::WeaponLinkDirectionV1::TwoWay;
        profile.directionSource="Meteor/Gripen C-D two-way direction: Saab 2010 future-missile-system-for-gripen; repository gripen XML identifies JAS39C; payload/timing are engineering assumptions";
    }
    WeaponBindingV1 b;b.owner=launcher.info.id;b.activity=weapon.origin.activity;b.target=weapon.origin.target;b.generation=weapon.info.generation;
    if(const auto* a=launcher.ms.activity(b.activity))b.controller=a->controller;
    b.channel=std::make_unique<ms::weapon::WeaponChannelV1>(profile,Rng::forVehicle(options_.seed,0x574C4E4Bull,weapon.info.id).next());
    if(weapon.guidance) {
        std::optional<ms::weapon::NavigationStrapdownConfigV2> selected;std::string error;
        if(!ms::weapon::navigationConfigurationV2(card,selected,error))throw std::runtime_error(error);
        const auto seed=Rng::forVehicle(options_.seed,0x574E4156ull,weapon.info.id).next();
        const auto& own=pool_->states()[weapon.slot];
        if(selected) {
            b.strapdownNavigation=std::make_unique<ms::weapon::NavigationStrapdownV2>(*selected,seed);
            if(!b.strapdownNavigation->initialize(nativeNavigationInputV2(own,simTime_,true)))
                throw std::runtime_error("Own strapdown navigation initialization refused native release pose");
            b.navigationCapturesV2.reset(nativeNavigationInputV2(own,simTime_,true));
        } else {
            ms::weapon::NavigationConfigV1 navigationConfig;navigationConfig.gnssAiding=card.id=="aim-120d";
            b.navigation=std::make_unique<ms::weapon::NavigationV1>(navigationConfig,seed);
            const auto input=nativeNavigationInputV2(own,simTime_,true);
            ms::weapon::NavigationInputV1 legacy;legacy.timeS=input.timeS;legacy.position=input.position;legacy.velocity=input.velocity;
            b.navigation->initialize(legacy);
        }
        b.navigationVelocity=nativeNavigationInputV2(own,simTime_,true).velocity;b.navigationTimeS=simTime_;
    }
    weaponLinksV1_.emplace(weapon.info.id,std::move(b));
}
ms::weapon::WeaponLinkPathV1 World::weaponLinkPathV1(const Entry& a,const Entry& b,const ms::WeaponLinkProfileV1& profile)const {
    const auto& x=pool_->states()[a.slot];const auto& y=pool_->states()[b.slot];
    ms::rf::PathOptions options;options.frequencyHz=profile.frequencyHz;options.ground=ground_.get();
    const auto path=ms::rf::computePath({x.latitudeRad,x.longitudeRad,x.altitudeMslM},{y.latitudeRad,y.longitudeRad,y.altitudeMslM},options);
    return {path.slantRangeM,!path.terrainMasked&&path.mode==ms::rf::PathMode::LineOfSight&&std::isfinite(path.slantRangeM)};
}
void World::prepareWeaponLinksV1() {
    for(auto& pair:weaponLinksV1_) {
        auto& b=pair.second;b.channel->advance(simTime_);
        const auto* w=entry(pair.first);const auto* launcher=entry(b.owner);
        if(!w||!launcher||w->weaponEnded||w->info.generation!=b.generation||simTime_<b.nextInputS||b.channel->profile().direction==ms::WeaponLinkDirectionV1::Unsupported)continue;
        b.nextInputS=simTime_+b.channel->profile().updatePeriodS;
        if(const auto entity=fusedWeaponInputV1(b.owner,b.target)) {
            const auto* designated=designation(b.target);
            if((designated&&designated->valid)||entity->state==ms::FusionStateV1::Lost)
                b.channel->sendInput(*entity,simTime_,weaponLinkPathV1(*launcher,*w,b.channel->profile()),&weaponLinkResourcesV1_[b.owner]);
        }
    }
}
void World::sampleWeaponLinkV1(const Entry& weapon,bool terminal) {
    const auto found=weaponLinksV1_.find(weapon.info.id);if(found==weaponLinksV1_.end())return;
    auto& b=found->second;
    // The physical terminal clock also bounds bindings whose receiver is gone
    // or whose communication generation closed. It does not invent a report.
    if(terminal&&b.endedS<0)b.endedS=simTime_;
    const auto* launcher=entry(b.owner);if(!launcher||weapon.info.generation!=b.generation||!b.channel->needsSample(simTime_,terminal))return;
    ms::WeaponFeedbackV1 report;report.weapon=weapon.info.id;report.sampleTimeS=simTime_;report.endOfFlight=terminal;
    report.tracking=weapon.origin.tracking;report.supported=weapon.origin.supported;
    report.active=weapon.origin.seeker=="active_radar"&&std::isfinite(weapon.origin.timeToActiveS)&&weapon.origin.timeToActiveS==0;
    if(std::isfinite(weapon.origin.timeToGoS)&&weapon.origin.timeToGoS>=0)report.timeToInterceptS=weapon.origin.timeToGoS;
    if(std::isfinite(weapon.origin.timeToActiveS)&&weapon.origin.timeToActiveS>=0)report.timeToActiveS=weapon.origin.timeToActiveS;
    // Only the onboard finite IMU/optional GNSS estimator produces telemetry.
    // Its coordinates, covariance and state epoch stay together.
    if(b.navigation||b.strapdownNavigation) {
        const auto nav=b.navigationEstimate();report.navigationValid=nav.valid;
        report.navigationTimeS=nav.timeS;report.navigationGnssAided=nav.gnssAided;
        const double p[3]={nav.position.x,nav.position.y,nav.position.z};
        const double v[3]={nav.velocity.x,nav.velocity.y,nav.velocity.z};
        if(nav.valid)for(int k=0;k<3;++k){report.stateEcef[2*k]=p[k];report.stateEcef[2*k+1]=v[k];}
        std::copy(nav.covariance.begin(),nav.covariance.end(),report.navigationCovariance);
    }
    b.channel->sample(report,simTime_,weaponLinkPathV1(weapon,*launcher,b.channel->profile()));
}
bool World::advanceWeaponNavigationReadV2(Entry& weapon,const sim::VehicleState& state,double time,bool gnss) {
    const auto found=weaponLinksV1_.find(weapon.info.id);if(found==weaponLinksV1_.end())return true;
    auto& b=found->second;if(!b.strapdownNavigation||weapon.weaponEnded)return true;
    if(b.navigationCaptureFailedV2)return false;
    try {
        if(!b.navigationCapturesV2.observe(*b.strapdownNavigation,nativeNavigationInputV2(state,time,gnss),true))b.navigationCaptureFailedV2=true;
    } catch(...) {b.navigationCaptureFailedV2=true;}
    // Native workers cannot throw through VehiclePool's worker thread. Stop
    // issuing modeled controls; the main boundary reports the exact refusal.
    if(b.navigationCaptureFailedV2)return false;
    const auto estimate=b.strapdownNavigation->estimate();b.navigationVelocity=estimate.velocity;b.navigationTimeS=estimate.timeS;
    return true;
}
sim::NativeStepObserverV1 World::weaponNavigationObserverV2(Entry& weapon,sim::FlightModel& model,double time) {
    const auto found=weaponLinksV1_.find(weapon.info.id);if(found==weaponLinksV1_.end()||weapon.weaponEnded)return {};
    auto& b=found->second;if(!b.strapdownNavigation)return {};
    const auto* native=dynamic_cast<const sim::JsbsimModel*>(&model);if(!native)return {};
    b.navigationNativeModelV2=&model;b.navigationNativeStartV2=time;b.navigationNativeDtV2=options_.dt;b.navigationNativeGnssV2=weapon.sensed.gnssValid;
    return {native,&b,[](void* context,unsigned substep,unsigned substeps) noexcept {
        auto& binding=*static_cast<WeaponBindingV1*>(context);
        // The final native state is consumed at the canonical next OUTER
        // control/World boundary. Avoid a differently rounded duplicate epoch.
        if(!substeps||substep>=substeps||binding.navigationCaptureFailedV2)return;
        try {
            sim::VehicleState state;binding.navigationNativeModelV2->state(state);
            const double epoch=binding.navigationNativeStartV2+binding.navigationNativeDtV2*static_cast<double>(substep)/static_cast<double>(substeps);
            if(!binding.navigationCapturesV2.observe(*binding.strapdownNavigation,nativeNavigationInputV2(state,epoch,binding.navigationNativeGnssV2),false))binding.navigationCaptureFailedV2=true;
        } catch(...) {binding.navigationCaptureFailedV2=true;}
    }};
}
std::optional<std::pair<double,double>> World::weaponNavigationControlEpochsV2(std::uint32_t weapon) const {
    const auto* e=entry(weapon);
    if(!e||!std::isfinite(e->finControllerTimeS)||!std::isfinite(e->finNavigationTimeS))return std::nullopt;
    return std::pair<double,double>{e->finControllerTimeS,e->finNavigationTimeS};
}
void World::advanceWeaponNavigationBoundaryV2() {
    for(auto& pair:weaponLinksV1_) {
        auto* weapon=entry(pair.first);
        if(!weapon||!pair.second.strapdownNavigation||weapon->weaponEnded)continue;
        if(!advanceWeaponNavigationReadV2(*weapon,pool_->states()[weapon->slot],simTime_,weapon->sensed.gnssValid))
            throw std::runtime_error("Own strapdown navigation refused captured native motion/control epoch");
    }
}
void World::advanceWeaponNavigationV1() {
    const double stepStart=simTime_-options_.dt*options_.frameSkip;
    for(auto& pair:weaponLinksV1_) {
        auto& b=pair.second;const auto* weapon=entry(pair.first);
        if(!weapon||(!b.navigation&&!b.strapdownNavigation)||weapon->weaponEnded)continue;
        if(b.strapdownNavigation)continue; // already consumed before SDK sensing
        const auto* sensed=sensedState(pair.first);
        const bool available=sensed&&sensed->gnssValid;
        const auto observe=[&](const sim::VehicleState& state,double time) {
            const double dt=time-b.navigationTimeS;if(!(dt>0))return;
            double v[3];ms::geom::velocityEcef(state,v);
            ms::weapon::NavigationInputV1 in;in.timeS=time;in.position={state.positionEcef[0],state.positionEcef[1],state.positionEcef[2]};
            in.velocity={v[0],v[1],v[2]};in.gnssAvailable=available;
            in.acceleration={(v[0]-b.navigationVelocity.x)/dt,(v[1]-b.navigationVelocity.y)/dt,(v[2]-b.navigationVelocity.z)/dt};
            if(b.navigation->advance(in)){b.navigationVelocity=in.velocity;b.navigationTimeS=time;}
        };
        for(std::size_t i=0;i<weapon->engagementStates.size()&&i<weapon->engagementPath.size();++i) {
            const double time=stepStart+weapon->engagementPath[i].offsetS;
            observe(weapon->engagementStates[i],time);
        }
        if(b.navigationTimeS<simTime_)observe(pool_->states()[weapon->slot],simTime_);
    }
}
void World::resetWeaponNavigationV1(Entry& weapon) {
    const auto found=weaponLinksV1_.find(weapon.info.id);if(found==weaponLinksV1_.end())return;
    auto& b=found->second;const auto input=nativeNavigationInputV2(weapon.working,simTime_,true);
    if(b.strapdownNavigation) {
        b.strapdownNavigation->reset();
        if(!b.strapdownNavigation->initialize(input))throw std::runtime_error("Own strapdown navigation reset refused native pose");
        b.navigationCapturesV2.reset(input);b.navigationCaptureFailedV2=false;
    } else if(b.navigation) {
        b.navigation->reset();ms::weapon::NavigationInputV1 legacy;
        legacy.timeS=input.timeS;legacy.position=input.position;legacy.velocity=input.velocity;b.navigation->initialize(legacy);
    }
    b.navigationVelocity=input.velocity;b.navigationTimeS=simTime_;
}
void World::finishWeaponLinksV1() {
    for(auto it=weaponLinksV1_.begin();it!=weaponLinksV1_.end();) {
        auto& b=it->second;
        const auto* weapon=entry(it->first);
        const bool live=weapon&&!weapon->weaponEnded;
        if(live)sampleWeaponLinkV1(*weapon);
        else if(b.endedS<0)b.endedS=simTime_;
        b.channel->advance(simTime_);
        // Onboard navigation belongs to the physical weapon, including a live
        // pose-reset generation. Losing its launcher only revokes radio access;
        // it cannot replace autonomous INS with truth or disable its reserve.
        // Once terminal, absent receivers need no retention; present receivers
        // retain bounded history regardless of the assessment duration.
        if(!live&&(!entry(b.owner)||(b.endedS>=0&&simTime_>b.endedS+b.channel->profile().retentionS)))
            it=weaponLinksV1_.erase(it);
        else ++it;
    }
    // Removed launchers cannot submit another transmission. Their resource
    // clocks have no onboard state and need not outlive the endpoint.
    for(auto it=weaponLinkResourcesV1_.begin();it!=weaponLinkResourcesV1_.end();) {
        if(!entry(it->first))it=weaponLinkResourcesV1_.erase(it);
        else ++it;
    }
}
ms::WeaponLinkCapabilityV1 World::weaponLinkCapabilityV1(std::uint32_t owner,std::uint32_t controller,std::uint32_t weapon)const {
    ms::WeaponLinkCapabilityV1 out;out.instance=weaponLinkInstance_;out.weapon=weapon;
    const auto it=weaponLinksV1_.find(weapon);
    // Removal revokes access immediately, before the next stepping/GC pass.
    // Actors receive the same refusal for absent owners and unknown bindings.
    if(it==weaponLinksV1_.end()||!entry(owner)){if(controller)out.reason=ms::WeaponLinkReasonV1::NotOwner;return out;}
    const auto& b=it->second;
    if(b.owner!=owner||(controller&&b.controller!=controller)){out.reason=ms::WeaponLinkReasonV1::NotOwner;return out;}
    if(const auto* live=entry(weapon);live&&live->info.generation!=b.generation){out.reason=ms::WeaponLinkReasonV1::Closed;return out;}
    out.activity=b.activity;out.profile=b.channel->profile();
    out.reason=out.profile.direction==ms::WeaponLinkDirectionV1::Unsupported?ms::WeaponLinkReasonV1::Unsupported:ms::WeaponLinkReasonV1::None;
    return out;
}
ms::WeaponAssessmentReceiptV1 World::assessWeaponV1(std::uint32_t owner,std::uint32_t controller,const ms::WeaponAssessmentRequestV1& q) {
    auto cap=weaponLinkCapabilityV1(owner,controller,q.weapon);
    if(q.instance!=weaponLinkInstance_)return {false,0,q.requestId,ms::WeaponLinkReasonV1::Closed};
    if(cap.reason!=ms::WeaponLinkReasonV1::None)return {false,0,q.requestId,cap.reason};
    if(const auto prior=weaponLinksV1_.at(q.weapon).channel->retry(q))return *prior;
    if(controller) {
        const auto* launcher=entry(owner);const auto* a=launcher?launcher->ms.activity(cap.activity):nullptr;
        if(!a||a->controller!=controller)return {false,0,q.requestId,ms::WeaponLinkReasonV1::NotOwner};
    }
    const auto* weapon=entry(q.weapon);const auto* launcher=entry(owner);
    if(!weapon||weapon->weaponEnded||!launcher)return {false,0,q.requestId,ms::WeaponLinkReasonV1::Expired};
    auto& b=weaponLinksV1_.at(q.weapon);
    return b.channel->assess(q,simTime_,weaponLinkPathV1(*launcher,*weapon,b.channel->profile()),&weaponLinkResourcesV1_[owner]);
}
bool World::cancelWeaponAssessmentV1(std::uint32_t owner,std::uint32_t controller,std::uint64_t instance,std::uint32_t weapon,std::uint64_t assessment) {
    if(instance!=weaponLinkInstance_||weaponLinkCapabilityV1(owner,controller,weapon).reason!=ms::WeaponLinkReasonV1::None)return false;
    auto& binding=weaponLinksV1_.at(weapon);
    ms::weapon::WeaponLinkPathV1 path;path.clear=false;
    const auto* live=entry(weapon);const auto* launcher=entry(owner);
    if(live&&launcher)path=weaponLinkPathV1(*launcher,*live,binding.channel->profile());
    return binding.channel->cancel(assessment,simTime_,path,&weaponLinkResourcesV1_[owner]);
}
ms::WeaponAssessmentStatusV1 World::weaponAssessmentStatusV1(std::uint32_t owner,std::uint32_t controller,std::uint32_t weapon,std::uint64_t assessment)const {
    const auto cap=weaponLinkCapabilityV1(owner,controller,weapon);
    if(cap.reason!=ms::WeaponLinkReasonV1::None){ms::WeaponAssessmentStatusV1 out;out.reason=cap.reason;return out;}
    return weaponLinksV1_.at(weapon).channel->status(assessment);
}
std::vector<ms::WeaponFeedbackV1> World::weaponFeedbackV1(std::uint32_t owner,std::uint32_t controller,std::uint32_t weapon,std::uint64_t assessment,std::uint64_t since)const {
    if(weaponLinkCapabilityV1(owner,controller,weapon).reason!=ms::WeaponLinkReasonV1::None)return {};
    return weaponLinksV1_.at(weapon).channel->feedback(assessment,since);
}
}
