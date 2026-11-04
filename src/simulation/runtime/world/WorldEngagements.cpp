// Native engagement lifecycle. JSBSim alone integrates weapon motion.
#include "simulation/runtime/world/World.h"
#include "simulation/mission_systems/components/registration/BuiltInProvidersV1.h"
#include "simulation/mission_systems/components/support/MeasuredSeekerV2.h"
#include "simulation/mission_systems/components/implementations/seeker/EngineeringSeekerV3.h"
#include "simulation/mission_systems/components/support/ReceiverConfigurationV1.h"
#include "simulation/flight/NativeThermalSourceV1.h"
#include "simulation/flight/GroundProvider.h"
#include "foundation/io/AssetResolver.h"
#include "foundation/core/Log.h"
#include "simulation/mission_systems/rf/Path.h"
#include "simulation/mission_systems/weapon/Fuze.h"
#include "simulation/mission_systems/weapon/Seeker.h"
#include "simulation/mission_systems/radar/Radar.h"
#include "foundation/core/Attitude.h"
#include <algorithm>
#include <cmath>
#include <set>

namespace fsim::session {
namespace {
using V = ms::weapon::V3;
V point(const double* p) { return {p[0], p[1], p[2]}; }
V velocity(const sim::VehicleState& s) { double v[3]; ms::geom::velocityEcef(s, v); return point(v); }
bool infrared(const std::string& seeker) { return seeker.find("infrared") != std::string::npos; }
double length(V v) { return std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z); }
V delta(V a, V b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
double dot(V a, V b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
double totalWeaponThrust(const sim::VehicleState& s) {
    double sum=0;
    for(int i=0;i<s.engineCount && i<sim::VehicleState::kMaxEngines;++i)sum+=s.thrustN[i];
    return sum;
}
ms::rf::Terminal terminal(V p) {
    const double horizontal=std::hypot(p.x,p.y);
    double latitude=std::atan2(p.z,horizontal*(1-.00669437999014)), n=6378137;
    for(int k=0;k<5;++k) {
        n=6378137/std::sqrt(1-.00669437999014*std::sin(latitude)*std::sin(latitude));
        latitude=std::atan2(p.z+.00669437999014*n*std::sin(latitude),horizontal);
    }
    return {latitude,std::atan2(p.y,p.x),horizontal/std::max(std::cos(latitude),1e-9)-n};
}
bool opticalClear(V from,V to,const sim::GroundProvider* ground) {
    if(!ground)return false;
    const double range=length(delta(to,from));
    const double spacing=std::clamp(ground->resolutionM(),30.,2000.);
    const unsigned count=static_cast<unsigned>(std::clamp(std::ceil(range/spacing),2.,2000.));
    for(unsigned i=1;i<count;++i) {
        const double w=static_cast<double>(i)/count;
        const auto p=terminal(ms::components::measured::add(from,ms::components::measured::scale(delta(to,from),w)));
        const double height=ground->pointHeightAboveEllipsoidM(p.latitudeRad,p.longitudeRad);
        if(!std::isfinite(height)||p.heightM<height)return false;
    }
    return true;
}
const ms::components::ReceiverConfigurationV2* physicalReceiver(const ms::components::SeekerProviderV1* provider,
    const ms::components::DescriptorV1* descriptor,ms::components::ReceiverConfigurationV2& scratch) {
    if(const auto* native=dynamic_cast<const ms::components::EngineeringSeekerV3*>(provider))
        if(const auto* configured=native->receiverConfigurationV2())return configured;
    if(!descriptor||!descriptor->bindings.count("receiver_model"))return nullptr;
    ms::components::DescriptorV1 normalized;std::string error;
    return ms::components::receiverConfigurationV2(*descriptor,normalized,scratch,error)?&scratch:nullptr;
}
const ms::components::ReticleReceiverConfigurationV1* reticleReceiver(const ms::components::SeekerProviderV1* provider,
    const ms::components::DescriptorV1* descriptor,ms::components::ReceiverConfigurationV3& scratch) {
    if(const auto* native=dynamic_cast<const ms::components::EngineeringSeekerV3*>(provider))
        if(const auto* configured=native->reticleReceiverConfigurationV1())return configured;
    if(!descriptor)return nullptr;
    const auto selection=descriptor->bindings.find("receiver_model");
    if(selection==descriptor->bindings.end()||selection->second!="thermal_conscan_reticle.v1")return nullptr;
    ms::components::DescriptorV1 scalar;std::string error;
    return ms::components::receiverConfigurationV3(*descriptor,scalar,scratch,error)&&scratch.reticle?&*scratch.reticle:nullptr;
}
ms::components::ThermalSceneFrameV1 thermalBackground(const ms::components::ReceiverScenePolicyV1& policy,
    const sim::VehicleState& own,double time) {
    ms::components::ThermalSceneFrameV1 frame;frame.timeS=time;
    frame.up={std::cos(own.latitudeRad)*std::cos(own.longitudeRad),std::cos(own.latitudeRad)*std::sin(own.longitudeRad),std::sin(own.latitudeRad)};
    const double radius=length(point(own.positionEcef));
    const double ratio=std::clamp((radius-std::max(0.,own.altitudeAglM))/radius,0.,1.);
    frame.groundHorizonCosine=-std::sqrt(std::max(0.,1-ratio*ratio));
    frame.skyRadianceWm2Sr=ms::components::ThermalImageReceiverV1::bandRadiance(policy.skyTemperatureK,1,policy.lowUm,policy.highUm);
    frame.groundRadianceWm2Sr=ms::components::ThermalImageReceiverV1::bandRadiance(policy.groundTemperatureK,1,policy.lowUm,policy.highUm);
    return frame;
}
void appendThermalFlare(ms::components::ThermalSceneFrameV1& frame,const ms::components::ReceiverScenePolicyV1& policy,
    V direction,V rate,double range,double age,std::size_t source) {
    ms::components::ThermalRadiancePatchV1 flare;flare.direction=direction;flare.rangeM=range;flare.directionRate=rate;
    const double radius=policy.flareInitialRadiusM+policy.flareExpansionMs*age;
    flare.widthRad=flare.heightRad=2*std::atan2(radius,range);
    const double temperature=policy.groundTemperatureK+(policy.flareTemperatureK-policy.groundTemperatureK)*std::exp(-age/policy.flareCoolingS);
    flare.radianceWm2Sr=ms::components::ThermalImageReceiverV1::bandRadiance(temperature,policy.flareEmissivity,policy.lowUm,policy.highUm);
    flare.transmission=std::exp(-policy.extinctionPerM*range);flare.pathRadianceWm2Sr=frame.skyRadianceWm2Sr;
    frame.patches.push_back(flare);frame.refereeAttributionIndices.push_back(source);
}
V bodyPoint(const sim::VehicleState& state,V body,const double cg[3]) {
    body=delta(body,point(cg));const auto* r=state.rotationBodyToEcef;
    return {state.positionEcef[0]+r[0]*body.x+r[1]*body.y+r[2]*body.z,
        state.positionEcef[1]+r[3]*body.x+r[4]*body.y+r[5]*body.z,
        state.positionEcef[2]+r[6]*body.x+r[7]*body.y+r[8]*body.z};
}
V directionRate(V relative,V relativeVelocity) {
    using namespace ms::components::measured;
    const double range=norm(relative);const auto ray=unit(relative);
    return range>0?scale(subtract(relativeVelocity,scale(ray,ms::components::measured::dot(relativeVelocity,ray))),1/range):V{};
}
void appendThermalAircraft(ms::components::ThermalSceneFrameV1& frame,const ms::components::ReceiverScenePolicyV1& policy,
    const sim::VehicleState& own,const sim::VehicleState& target,sim::FlightModel& model,
    const ms::damage::AttachedGeometryView& geometry,const ms::components::ImageExtentV2& extent,std::size_t source) {
    using namespace ms::components::measured;
    sim::NativeThermalSnapshotV1 temperatures;
    const auto* native=dynamic_cast<const sim::NativeThermalSourceV1*>(&model);
    if(!native||!native->nativeThermalSnapshotV1(temperatures)||!temperatures.ambientAvailable)return;
    double hottest=temperatures.ambientTemperatureK;
    for(std::size_t i=0;i<temperatures.engineGasTemperatureK.size();++i)
        if(temperatures.engineGasTemperatureKind[i]!=sim::NativeGasTemperatureKindV1::Unavailable)
            hottest=std::max(hottest,temperatures.engineGasTemperatureK[i]);
    const double skin=temperatures.ambientTemperatureK*(1+.2*policy.skinRecoveryFactor*target.mach*target.mach)+
        policy.engineHeatFraction*std::max(0.,hottest-temperatures.ambientTemperatureK);
    const V relative=delta(point(target.positionEcef),point(own.positionEcef));
    const double range=norm(relative);
    ms::components::ThermalRadiancePatchV1 body;body.direction=unit(relative);
    body.directionRate=directionRate(relative,delta(velocity(target),velocity(own)));
    body.widthRad=extent.widthRad;body.heightRad=extent.heightRad;body.rangeM=range;
    body.radianceWm2Sr=ms::components::ThermalImageReceiverV1::bandRadiance(skin,policy.bodyEmissivity,policy.lowUm,policy.highUm);
    body.transmission=std::exp(-policy.extinctionPerM*range);
    body.pathRadianceWm2Sr=ms::components::ThermalImageReceiverV1::bandRadiance(temperatures.ambientTemperatureK,1,policy.lowUm,policy.highUm);
    frame.patches.push_back(body);frame.refereeAttributionIndices.push_back(source);
    double cg[3];if(!model.cgFromOriginBodyM(cg))return;
    const V forward{target.rotationBodyToEcef[0],target.rotationBodyToEcef[3],target.rotationBodyToEcef[6]};
    for(const auto index:geometry.componentIndices()) {
        const auto& c=geometry.airframe().components[index];
        if(c.engine<0||static_cast<std::size_t>(c.engine)>=temperatures.engineGasTemperatureKind.size()||
            temperatures.engineGasTemperatureKind[static_cast<std::size_t>(c.engine)]==sim::NativeGasTemperatureKindV1::Unavailable)continue;
        const auto at=bodyPoint(target,{c.centre.x-c.half.x,c.centre.y,c.centre.z},cg);
        const auto offset=delta(at,point(own.positionEcef));const auto ray=unit(offset);
        const double visibility=std::max(0.,ms::components::measured::dot(forward,ray));if(!(visibility>0))continue;
        auto hot=body;hot.direction=ray;hot.rangeM=norm(offset);
        hot.directionRate=directionRate(offset,delta(velocity(target),velocity(own)));
        const double radius=std::max(c.half.y,c.half.z)*std::sqrt(policy.exhaustAreaScale);
        hot.widthRad=2*std::atan2(radius*visibility,hot.rangeM);hot.heightRad=2*std::atan2(radius,hot.rangeM);
        hot.radianceWm2Sr=ms::components::ThermalImageReceiverV1::bandRadiance(
            temperatures.engineGasTemperatureK[static_cast<std::size_t>(c.engine)],policy.exhaustEmissivity,policy.lowUm,policy.highUm);
        hot.transmission=std::exp(-policy.extinctionPerM*hot.rangeM);
        if(hot.widthRad>0&&hot.heightRad>0){frame.patches.push_back(hot);frame.refereeAttributionIndices.push_back(source);}
    }
}
}

ms::components::SeekerNavigationPoseV1 World::weaponNavigationPoseV1(const Entry& e) const {
    ms::components::SeekerNavigationPoseV1 result;
    const auto at=weaponLinksV1_.find(e.info.id);if(at==weaponLinksV1_.end())return result;
    const auto estimate=at->second.navigationEstimate();if(!estimate.valid)return result;
    result.timeS=estimate.timeS;result.position=estimate.position;result.velocity=estimate.velocity;
    if(at->second.strapdownNavigation) {
        const auto q=at->second.strapdownNavigation->attitudeBodyToEcef();
        const double w=q[0],x=q[1],y=q[2],z=q[3];
        result.bodyToEcef={{{1-2*(y*y+z*z),2*(x*y+w*z),2*(x*z-w*y)},
            {2*(x*y-w*z),1-2*(x*x+z*z),2*(y*z+w*x)},
            {2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)}}};
    } else {
        if(e.payloadProfile)if(const auto* d=e.payloadProfile->find("seeker"))if(d->bindings.count("receiver_model"))return {};
        const auto& s=pool_->states()[e.slot];
        result.bodyToEcef={{{s.rotationBodyToEcef[0],s.rotationBodyToEcef[3],s.rotationBodyToEcef[6]},
            {s.rotationBodyToEcef[1],s.rotationBodyToEcef[4],s.rotationBodyToEcef[7]},
            {s.rotationBodyToEcef[2],s.rotationBodyToEcef[5],s.rotationBodyToEcef[8]}}};
    }
    result.valid=true;return result;
}

void World::recordEngagement(const std::string& kind, const Entry& e, std::uint32_t target,
                             const std::string& reason, bool destroyed) {
    ms::EngagementEvent event;
    event.serial = nextCombatSerial_++;
    event.timeS = simTime_;
    event.kind = kind; event.reason = reason;
    event.entity = e.info.id; event.launcher = e.info.launcher; event.target = target;
    const auto& state = pool_->states()[e.slot];
    std::copy_n(state.positionEcef, 3, event.positionEcef);
    ms::geom::velocityEcef(state, event.velocityEcef);
    event.destroyed = destroyed;
    const auto* targetEntry=entry(target);
    recordWeaponEventGenerationV1(event,e.info.generation,targetEntry?targetEntry->info.generation:0);
    combatEvents_.push_back(std::move(event));
}

ms::Reason World::dispense(std::uint32_t vehicle, bool flare) {
    Entry* e = entry(vehicle);
    if (!e) return ms::Reason::UnknownVehicle;
    bool operating = false, installed = false;
    for (const auto& p : e->ms.payloads()) if (p.family == ms::Family::Countermeasures) {
        installed = true;
        operating = operating || p.state == ms::PayloadState::Operate;
    }
    if (!installed) return ms::Reason::NotSupported;
    if (!operating) return ms::Reason::PayloadOff;
    if (!e->dispenser) e->dispenser = std::make_unique<ms::countermeasure::Dispenser>(vehicle, 30, 15);
    const auto kind = flare ? ms::countermeasure::Kind::Flare : ms::countermeasure::Kind::Chaff;
    if (!e->dispenser->remaining(kind)) return ms::Reason::Expended;
    if (nextDecoyId_ >= kDesignations) return ms::Reason::NotSupported;
    const auto& s = pool_->states()[e->slot];
    const V v = velocity(s);
    if (!e->dispenser->dispense(kind, simTime_, {s.positionEcef[0],s.positionEcef[1],s.positionEcef[2]},
                              {v.x,v.y,v.z}, nextDecoyId_)) return ms::Reason::StoreBusy;
    ++nextDecoyId_;
    recordEngagement(flare ? "flare" : "chaff", *e);
    return ms::Reason::None;
}

ms::CountermeasureInventory World::countermeasures(std::uint32_t vehicle) const {
    const Entry* e = entry(vehicle);
    if (!e) return {};
    if (e->dispenser) return {e->dispenser->remaining(ms::countermeasure::Kind::Chaff),
                              e->dispenser->remaining(ms::countermeasure::Kind::Flare)};
    for (const auto& p : e->ms.payloads()) if (p.family == ms::Family::Countermeasures) return {30,15};
    return {};
}

std::vector<ms::DecoyState> World::decoys() const {
    std::vector<ms::DecoyState> out;
    for (const auto* dispenser : dispensers()) for (const auto& d : dispenser->decoys()) {
        ms::DecoyState state;
        state.id=d.id; state.owner=d.owner; state.flare=d.kind==ms::countermeasure::Kind::Flare;
        state.ageS=std::max(0.0,simTime_-d.launchTimeS);
        state.positionEcef[0]=d.position.x; state.positionEcef[1]=d.position.y; state.positionEcef[2]=d.position.z;
        out.push_back(state);
    }
    return out;
}

std::vector<ms::EngagementEvent> World::engagementEvents(std::uint64_t since) const {
    std::vector<ms::EngagementEvent> out;
    for (const auto& event : combatEvents_) if (event.serial>since) out.push_back(event);
    return out;
}

void World::prepareEngagements() {
    const double dt = options_.dt*options_.frameSkip;
    decoyTrajectories_.clear();
    for (auto& e : entries_) if (e && e->guidance && !e->weaponEnded) {
        if(e->arrestedBurst)continue;
        const auto& s=pool_->states()[e->slot];
        const auto spec=e->guidance->spec();
        const bool heat=infrared(spec.seeker);
        ms::weapon::Input in;
        const V physicalPosition=point(s.positionEcef),physicalVelocity=velocity(s);
        in.position=physicalPosition;in.velocity=physicalVelocity;in.dtS=dt;
        if(const auto binding=weaponLinksV1_.find(e->info.id);binding!=weaponLinksV1_.end()) {
            const auto nav=binding->second.navigationEstimate();
            if(nav.valid){in.position=nav.position;in.velocity=nav.velocity;}
        }
        in.boresight=V{s.rotationBodyToEcef[0],s.rotationBodyToEcef[3],s.rotationBodyToEcef[6]};
        in.ageS=simTime_-e->info.launchTimeS;
        in.launchCue=heat?e->launchSeekerCue:0;
        in.up={std::cos(s.latitudeRad)*std::cos(s.longitudeRad),std::cos(s.latitudeRad)*std::sin(s.longitudeRad),std::sin(s.latitudeRad)};
        if(!e->seekerProvider) {
            // A formally uninstalled terminal seeker cannot enumerate candidate
            // truth or refresh the authorized launch-time aimpoint.
            const auto output=e->guidance->update(in);
            e->guidanceDecisionTimeS=simTime_;
            e->origin.phase=output.phase;e->origin.holding=0;
            e->origin.tracking=false;e->origin.supported=false;
            e->origin.timeToGoS=output.timeToGoS;e->origin.timeToActiveS=output.timeToActiveS;
            e->origin.burning=e->plantProviders.propulsion && e->plantProviders.propulsion->burning(totalWeaponThrust(s));
            e->guidanceDemand[0]=output.acceleration.x;e->guidanceDemand[1]=output.acceleration.y;e->guidanceDemand[2]=output.acceleration.z;
            continue;
        }
        const auto& seeker=e->seekerProvider->config();
        const bool finiteSensor=seeker.engineering.version==3;
        ms::components::ReceiverConfigurationV2 receiverScratch;
        const auto* seekerDescriptor=e->payloadProfile?e->payloadProfile->find("seeker"):nullptr;
        const bool physicalDeclared=seekerDescriptor&&seekerDescriptor->bindings.count("receiver_model");
        const auto* receiverEnvelope=physicalReceiver(e->seekerProvider.get(),seekerDescriptor,receiverScratch);
        const auto* receiver=receiverEnvelope?&receiverEnvelope->single:nullptr;
        ms::components::ReceiverConfigurationV3 reticleScratch;
        const auto* reticle=reticleReceiver(e->seekerProvider.get(),seekerDescriptor,reticleScratch);
        const auto* thermalScenePolicy=reticle?&reticle->scene:receiver?&receiver->scene:nullptr;
        const bool dualThermal=receiverEnvelope&&receiverEnvelope->dual&&receiverEnvelope->secondBandScene;
        const bool thermalReceiver=reticle||(receiver&&(receiver->kind==ms::components::PhysicalReceiverKindV1::Image||receiver->kind==ms::components::PhysicalReceiverKindV1::Quadrant));
        const bool radioReceiver=receiver&&receiver->kind==ms::components::PhysicalReceiverKindV1::Radio;
        const auto* sceneClock=dynamic_cast<const ms::components::SeekerSceneClockV1*>(e->seekerProvider.get());
        const bool captureGroundScene=!sceneClock||sceneClock->sceneCaptureDueV1(simTime_);
        std::vector<ms::components::SeekerSceneV3> sensorScenes;
        const auto* d=designation(e->origin.target);
        const Entry* launcher=entry(e->info.launcher);
        const ms::radar::Radar* illuminator=nullptr;
        std::uint64_t illuminatedTrack=0;
        std::optional<ms::FusedEntityV1> deliveredInput;
        const auto link=weaponLinksV1_.find(e->info.id);
        if(link!=weaponLinksV1_.end()&&e->info.generation==link->second.generation)deliveredInput=link->second.channel->input();
        if(deliveredInput) {
            const auto& f=*deliveredInput;const double age=simTime_-f.timeS;
            in.supported=true;
            in.estimate=ms::weapon::Observation{e->origin.target,
                {f.stateEcef[0]+age*f.stateEcef[1],f.stateEcef[2]+age*f.stateEcef[3],f.stateEcef[4]+age*f.stateEcef[5]},
                {f.stateEcef[1],f.stateEcef[3],f.stateEcef[5]},false};
        }
        if(d && launcher && d->valid) {
            if(d->fusedEntityV1) {
                if(const auto fused=fusedWeaponInputV1(d->owner,d->id))for(const auto& source:fused->sources) {
                    if(source.kind!=ms::FusionSourceKindV1::LocalRadar||source.payload>65535)continue;
                    const auto payload=static_cast<std::uint16_t>(source.payload);
                    const auto status=launcher->ms.radarStatus(payload);
                    if(status.illuminating&&status.designatedTrack==source.track) {
                        illuminator=launcher->ms.radar(payload);illuminatedTrack=source.track;in.illuminated=true;break;
                    }
                }
            } else {
                const auto r=launcher->ms.radarStatus(d->payload);
                in.illuminated=d->source==comm::kNoSystem&&r.illuminating&&r.designatedTrack==d->track;
                if(in.illuminated){illuminator=launcher->ms.radar(d->payload);illuminatedTrack=d->track;}
            }
        }
        const bool semi=spec.seeker=="semi_active_radar";
        auto thermalScene=thermalReceiver?thermalBackground(*thermalScenePolicy,s,simTime_):ms::components::ThermalSceneFrameV1{};
        auto secondThermalScene=dualThermal?thermalBackground(*receiverEnvelope->secondBandScene,s,simTime_):ms::components::ThermalSceneFrameV1{};
        ms::components::RadioSceneFrameV1 radioScene;radioScene.timeS=simTime_;radioScene.up=in.up;
        if(radioReceiver){radioScene.frequencyHz=semi&&illuminator?illuminator->spec().frequencyHz:receiver->radio.frequencyHz;
            radioScene.illuminationAvailable=semi&&in.illuminated&&illuminator&&launcher;
            if(radioScene.illuminationAvailable){radioScene.illuminationWaveform=illuminator->waveform();
                radioScene.illuminationClock=illuminator->illuminationClockV1();}
            radioScene.altitudeM=std::max(0.,s.altitudeAglM);}
        const auto illuminatedCandidate=[&](V position) {
            if (!semi) return true;
            if (!illuminator || !in.illuminated || !launcher || !illuminator->illuminating()
                || illuminator->designated()!=illuminatedTrack) return false;
            const auto& transmitter=pool_->states()[launcher->slot];
            const V ray=delta(position,point(transmitter.positionEcef));
            const double raw[3]={ray.x,ray.y,ray.z}; double body[3]; ms::geom::toBody(transmitter,raw,body);
            if (!e->seekerProvider->illuminated(body,illuminator->beamAzimuthRad(),illuminator->beamElevationRad(),
                    illuminator->pattern().beamwidthRad(false),illuminator->pattern().beamwidthRad(true))) return false;
            ms::rf::PathOptions path; path.ground=ground_.get(); path.frequencyHz=illuminator->spec().frequencyHz;
            if (!std::isfinite(path.frequencyHz) || path.frequencyHz<=0) return false;
            return !ms::rf::computePath({transmitter.latitudeRad,transmitter.longitudeRad,transmitter.altitudeMslM},terminal(position),path).terrainMasked;
        };
        const auto sceneFor=[&](V position,double rcs)->std::optional<ms::components::SeekerSceneV3> {
            ms::components::SeekerSceneV3 scene;scene.rangeM=length(delta(position,physicalPosition));scene.rcsM2=rcs;
            ms::rf::PathOptions path;path.ground=ground_.get();
            path.frequencyHz=semi&&illuminator?illuminator->spec().frequencyHz:seeker.engineering.frequencyHz;
            const auto pathResult=ms::rf::computePath({s.latitudeRad,s.longitudeRad,s.altitudeMslM},terminal(position),path);
            if(pathResult.terrainMasked||!std::isfinite(pathResult.totalDb)||!std::isfinite(pathResult.freeSpaceDb))return std::nullopt;
            scene.receiverExcessLossDb=heat?0:std::max(0.,pathResult.totalDb-pathResult.freeSpaceDb);
            if(semi&&illuminator&&launcher) {
                const auto& tx=pool_->states()[launcher->slot];
                const auto ray=delta(position,point(tx.positionEcef));const double raw[3]={ray.x,ray.y,ray.z};double body[3];ms::geom::toBody(tx,raw,body);
                const auto transmitter=ms::rf::computePath({tx.latitudeRad,tx.longitudeRad,tx.altitudeMslM},terminal(position),path);
                if(transmitter.terrainMasked||!std::isfinite(transmitter.totalDb)||!std::isfinite(transmitter.freeSpaceDb))return std::nullopt;
                scene.transmitterRangeM=length(ray);scene.transmitterPowerW=illuminator->spec().averagePowerW;
                const double az=illuminator->beamAzimuthRad(),el=illuminator->beamElevationRad();
                const V beam{std::cos(el)*std::cos(az),std::cos(el)*std::sin(az),-std::sin(el)};
                const V direction=ms::components::measured::unit(V{body[0],body[1],body[2]});
                if(illuminator->spec().aperture.electronic) {
                    const double spacing=2*3.14159265358979323846/illuminator->spec().azimuthFaces;
                    const double centre=std::round(std::atan2(beam.y,beam.x)/spacing)*spacing;
                    const double c=std::cos(centre),sn=std::sin(centre);
                    const double local[3]={c*direction.x+sn*direction.y,-sn*direction.x+c*direction.y,direction.z};
                    scene.transmitterGainLinear=illuminator->pattern().gain(local,-sn*beam.x+c*beam.y,beam.z);
                } else {
                    const auto y=ms::components::measured::unit(V{-beam.y,beam.x,0});
                    const auto z=ms::components::measured::cross(beam,y);
                    const double face[3]={dot(direction,beam),dot(direction,y),dot(direction,z)};
                    scene.transmitterGainLinear=illuminator->pattern().gain(face);
                }
                scene.transmitterExcessLossDb=std::max(0.,transmitter.totalDb-transmitter.freeSpaceDb);
                scene.transmitterFrequencyHz=illuminator->spec().frequencyHz;
            }
            return scene;
        };
        // Seeker observations, not omniscient updates: range, aspect, gimbal and Doppler gates.
        // basis: assumed. IR aspect/afterburner factors, 28 m/s notch and decoy response are game estimates.
        for (const auto& candidate : entries_) if (candidate && candidate->info.role==ms::Role::Aircraft &&
                candidate->info.id!=e->info.launcher && candidate->info.id!=e->info.id) {
            const auto& target=pool_->states()[candidate->slot];
            if (target.crashed || target.diverged) continue;
            if (!illuminatedCandidate(point(target.positionEcef))) continue;
            const V r=delta(point(target.positionEcef),physicalPosition);
            const double range=length(r);
            if (range<seeker.minimumRangeM || (!finiteSensor&&!heat&&range>spec.acquireM)) continue;
            const V tv=velocity(target);
            double acquire=spec.acquireM,signal=seeker.aircraftSignal,rcs=seeker.radarRcsReferenceM2;
            if (heat) {
                const V targetForward{target.rotationBodyToEcef[0],target.rotationBodyToEcef[3],target.rotationBodyToEcef[6]};
                const double aspect=finiteSensor?ms::components::EngineeringSeekerV3::sourceAspectCosineV3(r,targetForward):dot(r,tv)/(range*std::max(length(tv),seeker.aspectSpeedFloorMs));
                acquire=e->seekerProvider->infraredRange(spec.acquireM,aspect,target.afterburner[0]>seeker.afterburnerThreshold);
                if(finiteSensor&&spec.acquireM>0)signal*=std::pow(acquire/spec.acquireM,2);
            } else {
                ms::RcsQuery query;
                if(finiteSensor)query.frequencyHz=semi&&illuminator?illuminator->spec().frequencyHz:seeker.engineering.frequencyHz;
                const auto cross=radarCrossSection(e->info.id,candidate->info.id,query);
                if(cross&&cross->modelled){rcs=cross->sigmaM2;if(!finiteSensor)acquire=e->seekerProvider->radarRange(acquire,cross->sigmaM2,true);}
                if (!radioReceiver&&e->seekerProvider->notched(dot(r,in.up)/range,dot(tv,r)/range)) continue;
            }
            if (!finiteSensor&&range>acquire) continue;
            // Terrain masking and the curved radio horizon also apply to the seeker's own sensor.
            ms::RadioPathOptions path;
            if (semi && illuminator) path.frequencyHz=illuminator->spec().frequencyHz;
            if(finiteSensor) {
                std::optional<ms::components::SeekerSceneV3> scene;
                if(thermalReceiver){if(!opticalClear(physicalPosition,point(target.positionEcef),ground_.get()))continue;
                    scene=ms::components::SeekerSceneV3{};scene->rangeM=range;}
                else scene=sceneFor(point(target.positionEcef),rcs);
                if(!scene)continue;
                sensorScenes.push_back(*scene);
            } else {
                const auto propagation=radioPath(e->info.id,candidate->info.id,path);
                if(propagation&&propagation->terrainMasked)continue;
            }
            in.visible.push_back({candidate->info.id,point(target.positionEcef),tv,false,signal,acquire});
        }
        for (const auto* dispenser : dispensers()) for (const auto& decoy : dispenser->decoys()) {
            if (heat != (decoy.kind==ms::countermeasure::Kind::Flare)) continue;
            const V p{decoy.position.x,decoy.position.y,decoy.position.z}, v{decoy.velocity.x,decoy.velocity.y,decoy.velocity.z};
            if (!illuminatedCandidate(p)) continue;
            const V r=delta(p,physicalPosition); const double range=length(r);
            if (range<seeker.minimumRangeM || (!finiteSensor&&range>spec.acquireM) || (!heat&&!radioReceiver && e->seekerProvider->notched(dot(r,in.up)/range,dot(v,r)/range))) continue;
            ms::rf::PathOptions path; path.ground=ground_.get();
            if (semi && illuminator) path.frequencyHz=illuminator->spec().frequencyHz;
            std::optional<ms::components::SeekerSceneV3> scene;
            if(finiteSensor){
                if(thermalReceiver){if(!opticalClear(physicalPosition,p,ground_.get()))continue;scene=ms::components::SeekerSceneV3{};scene->rangeM=range;}
                else scene=sceneFor(p,seeker.radarRcsReferenceM2);
                if(!scene)continue;
            }
            else if(ms::rf::computePath({s.latitudeRad,s.longitudeRad,s.altitudeMslM},terminal(p),path).terrainMasked)continue;
            // basis: assumed. Relative signature bloom/fade for deterministic seeker discrimination.
            const double age=std::max(0.0,simTime_-decoy.launchTimeS);
            const double signal=e->seekerProvider->decoySignal(age,heat);
            if(signal>0){in.visible.push_back({decoy.id,p,v,true,signal});if(scene)sensorScenes.push_back(*scene);}
        }
        const auto old=e->origin;
        ms::weapon::Output output;
        if(auto* sensor=dynamic_cast<ms::components::SeekerExecutionV2*>(e->seekerProvider.get())) {
            ms::components::SeekerFrameV2 frame;
            frame.timeS=simTime_;frame.ageS=in.ageS;frame.dtS=in.dtS;
            frame.position=in.position;frame.velocity=in.velocity;frame.up=in.up;frame.boresight=*in.boresight;
            frame.receiverSeed=Rng::forVehicle(options_.seed,0x53454E53ull,e->info.id).next();
            frame.bodyAxes=std::array<V,3>{{{s.rotationBodyToEcef[0],s.rotationBodyToEcef[3],s.rotationBodyToEcef[6]},
                {s.rotationBodyToEcef[1],s.rotationBodyToEcef[4],s.rotationBodyToEcef[7]},
                {s.rotationBodyToEcef[2],s.rotationBodyToEcef[5],s.rotationBodyToEcef[8]}}};
            frame.supported=in.supported;frame.illuminated=in.illuminated;
            frame.configuredIlluminator=illuminator&&launcher?launcher->info.id:0;
            if(spec.launcherUpdates&&deliveredInput) {
                const auto& f=*deliveredInput;
                frame.launcherEstimate=ms::components::LauncherEstimateV2{
                    {f.stateEcef[0],f.stateEcef[2],f.stateEcef[4]},
                    {f.stateEcef[1],f.stateEcef[3],f.stateEcef[5]},f.timeS};
            }
            std::vector<std::uint32_t> attributions;
            for(std::size_t index=0;index<in.visible.size();++index) {
                const auto& observation=in.visible[index];
                ms::components::SeekerSampleV2 sample;
                const auto relative=delta(observation.position,physicalPosition);const auto range=length(relative);
                sample.direction=ms::components::measured::unit(relative);sample.signal=observation.signal;
                if(spec.seeker=="active_radar") {
                    sample.rangeM=range;
                    sample.rangeRateMs=dot(delta(observation.velocity,physicalVelocity),sample.direction);
                }
                if(spec.seeker=="imaging_infrared")if(const auto* target=entry(observation.id))
                    sample.imageExtent=projectedSeekerExtent(*target,s);
                if(semi) {
                    sample.illuminator=frame.configuredIlluminator;
                    // Both actual path queries have passed above; no unrelated
                    // source or unilluminated candidate enters this snapshot.
                    sample.transmitterPathClear=true;sample.receiverPathClear=true;
                }
                if(finiteSensor&&index<sensorScenes.size())sample.scene=sensorScenes[index];
                if(thermalReceiver) {
                    if(const auto* target=entry(observation.id);target) {
                        if(const auto geometry=seekerGeometryV1(*target))if(const auto extent=projectedSeekerExtent(*target,s)) {
                            appendThermalAircraft(thermalScene,*thermalScenePolicy,s,
                                pool_->states()[target->slot],pool_->vehicle(target->slot),*geometry,*extent,index);
                            if(dualThermal)appendThermalAircraft(secondThermalScene,*receiverEnvelope->secondBandScene,s,
                                pool_->states()[target->slot],pool_->vehicle(target->slot),*geometry,*extent,index);
                        }
                    } else for(const auto* dispenser:dispensers())for(const auto& decoy:dispenser->decoys())if(decoy.id==observation.id) {
                        const double age=std::max(0.,simTime_-decoy.launchTimeS);
                        const auto rate=directionRate(relative,delta(observation.velocity,physicalVelocity));
                        appendThermalFlare(thermalScene,*thermalScenePolicy,sample.direction,rate,range,age,index);
                        if(dualThermal)appendThermalFlare(secondThermalScene,*receiverEnvelope->secondBandScene,sample.direction,rate,range,age,index);
                    }
                }
                if(radioReceiver&&sample.scene) {
                    const auto& physical=*sample.scene;ms::components::RadioEchoSceneV1 echo;echo.direction=sample.direction;
                    echo.receiverRangeM=range;echo.transmitterRangeM=physical.transmitterRangeM;
                    echo.delayRangeM=semi?.5*(physical.transmitterRangeM+range):range;
                    const double wavelength=299792458./radioScene.frequencyHz;
                    echo.dopplerHz=dot(delta(physicalVelocity,observation.velocity),sample.direction)/wavelength;
                    if(semi&&launcher) {
                        const auto& tx=pool_->states()[launcher->slot];
                        const auto txRay=ms::components::measured::unit(delta(observation.position,point(tx.positionEcef)));
                        echo.dopplerHz+=dot(delta(velocity(tx),observation.velocity),txRay)/wavelength;
                    } else echo.dopplerHz*=2;
                    echo.rcsM2=physical.rcsM2*(observation.decoy?observation.signal:1);
                    echo.receiverExcessLossDb=physical.receiverExcessLossDb;echo.transmitterExcessLossDb=physical.transmitterExcessLossDb;
                    echo.transmitterPowerW=physical.transmitterPowerW;echo.transmitterGainLinear=physical.transmitterGainLinear;
                    radioScene.echoes.push_back(echo);radioScene.refereeAttributionIndices.push_back(index);
                }
                frame.samples.push_back(sample);attributions.push_back(observation.id);
            }
            e->seekerAttributionsV3[simTime_]=std::move(attributions);
            ms::components::SeekerResultV2 result;
            const auto navigation=weaponNavigationPoseV1(*e);
            if(thermalReceiver) {
                thermalScene.navigation=navigation;
                if(reticle) {
                    if(auto* detector=dynamic_cast<ms::components::ThermalReticleSeekerExecutionV1*>(e->seekerProvider.get()))
                        result=detector->sampleThermalReticleV1(frame,thermalScene);
                } else if(dualThermal) {
                    secondThermalScene.navigation=navigation;
                    if(auto* detector=dynamic_cast<ms::components::ThermalDualBandSeekerExecutionV1*>(e->seekerProvider.get()))
                        result=detector->sampleThermalDualBandV1(frame,{{thermalScene,secondThermalScene}});
                } else if(auto* detector=dynamic_cast<ms::components::ThermalSeekerExecutionV1*>(e->seekerProvider.get()))result=detector->sampleThermalV1(frame,thermalScene);
            } else if(radioReceiver) {
                radioScene.navigation=navigation;
                // Same bounded physical ground-patch quadrature as the native
                // radar: explicit backscatter, actual terrain height and own
                // motion. Patches have no target/referee identity.
                const double h=std::max(1.,s.altitudeAglM),earth=length(physicalPosition)-h;
                const double maximum=std::min(receiver->scene.groundRangeM,std::sqrt(std::max(0.,(earth+h)*(earth+h)-earth*earth)));
                const auto up=ms::components::measured::unit(in.up);
                auto east=ms::components::measured::unit(ms::components::measured::cross(V{0,0,1},up));
                if(length(east)<.5)east=(*frame.bodyAxes)[1];
                const auto north=ms::components::measured::cross(up,east);
                // Clutter paths are consumed only at an actual detector exposure.
                // Keep the scene epoch/navigation and SARH illumination clock
                // every control step; servo and delivery timing are unchanged.
                for(unsigned band=0;captureGroundScene&&band<receiver->scene.groundRangeBands&&maximum>h;++band) {
                    const double dr=(maximum-h)/receiver->scene.groundRangeBands,range=h+(band+.5)*dr;
                    const double down=std::clamp(((earth+h)*(earth+h)+range*range-earth*earth)/(2*(earth+h)*range),0.,1.);
                    const double horizontal=std::sqrt(1-down*down),grazing=std::clamp(((earth+h)*(earth+h)-range*range-earth*earth)/(2*earth*range),0.,1.);
                    for(unsigned az=0;az<receiver->scene.groundAzimuthSamples;++az) {
                        const double phi=2*3.14159265358979323846*(az+.5)/receiver->scene.groundAzimuthSamples;
                        const auto ray=ms::components::measured::add(ms::components::measured::scale(up,-down),
                            ms::components::measured::add(ms::components::measured::scale(north,horizontal*std::cos(phi)),ms::components::measured::scale(east,horizontal*std::sin(phi))));
                        auto at=ms::components::measured::add(physicalPosition,ms::components::measured::scale(ray,range));auto place=terminal(at);
                        const double terrain=ground_?ground_->pointHeightAboveEllipsoidM(place.latitudeRad,place.longitudeRad):0;
                        if(!std::isfinite(terrain))continue;
                        double xyz[3];ms::rf::geodeticToEcef(place.latitudeRad,place.longitudeRad,terrain,xyz);at=point(xyz);
                        const double area=2*3.14159265358979323846*range*dr/receiver->scene.groundAzimuthSamples;
                        const double sigma=(terrain<=0?receiver->scene.groundSigma0Sea:receiver->scene.groundSigma0Land)*std::pow(grazing,receiver->scene.groundGrazingExponent)*area;
                        const auto physical=sceneFor(at,sigma);if(!physical)continue;
                        ms::components::RadioEchoSceneV1 echo;echo.direction=ms::components::measured::unit(delta(at,physicalPosition));echo.receiverRangeM=physical->rangeM;
                        echo.transmitterRangeM=physical->transmitterRangeM;echo.delayRangeM=semi?.5*(echo.receiverRangeM+echo.transmitterRangeM):echo.receiverRangeM;
                        const double wavelength=299792458./radioScene.frequencyHz;echo.dopplerHz=dot(physicalVelocity,echo.direction)/wavelength;
                        if(semi&&launcher){const auto& tx=pool_->states()[launcher->slot];echo.dopplerHz+=dot(velocity(tx),ms::components::measured::unit(delta(at,point(tx.positionEcef))))/wavelength;}
                        else echo.dopplerHz*=2;
                        echo.rcsM2=sigma;echo.transmitterPowerW=physical->transmitterPowerW;echo.transmitterGainLinear=physical->transmitterGainLinear;
                        echo.receiverExcessLossDb=physical->receiverExcessLossDb;echo.transmitterExcessLossDb=physical->transmitterExcessLossDb;
                        radioScene.groundPatches.push_back(echo);
                    }
                }
                if(auto* detector=dynamic_cast<ms::components::RadioSeekerExecutionV1*>(e->seekerProvider.get()))result=detector->sampleRadioV1(frame,radioScene);
            } else if(!physicalDeclared)result=sensor->sample(frame);
            e->seekerResultV2=result;
            if(auto* controller=dynamic_cast<ms::components::MeasuredGuidanceV2*>(e->guidance.get())) {
                auto guidanceFrame=frame;
                if(thermalReceiver||radioReceiver){guidanceFrame.position=navigation.position;guidanceFrame.velocity=navigation.velocity;
                    guidanceFrame.boresight=navigation.bodyToEcef[0];guidanceFrame.bodyAxes=navigation.bodyToEcef;}
                output=controller->updateMeasured(guidanceFrame,result);
            }
            // Truth attribution is outside the sensor and guidance contracts.
            const auto scene=e->seekerAttributionsV3.find(result.measurementTimeS.value_or(simTime_));
            output.holding=result.attributionIndex&&scene!=e->seekerAttributionsV3.end()&&*result.attributionIndex<scene->second.size()
                ?scene->second[*result.attributionIndex]:result.tracking?0:old.holding;
            const double retention=std::max(3.,seeker.engineering.processingDelayS+seeker.engineering.memoryS+2*seeker.engineering.samplePeriodS);
            while(!e->seekerAttributionsV3.empty()&&(e->seekerAttributionsV3.begin()->first<simTime_-retention||e->seekerAttributionsV3.size()>2048))
                e->seekerAttributionsV3.erase(e->seekerAttributionsV3.begin());
        } else output=e->guidance->update(in);
        e->guidanceDecisionTimeS=simTime_;
        e->origin.phase=output.phase; e->origin.holding=output.holding;
        e->origin.tracking=output.tracking; e->origin.supported=output.supported;
        e->origin.timeToGoS=output.timeToGoS; e->origin.timeToActiveS=output.timeToActiveS;
        e->origin.burning=e->plantProviders.propulsion ? e->plantProviders.propulsion->burning(totalWeaponThrust(s)) : totalWeaponThrust(s)>.1;
        e->guidanceDemand[0]=output.acceleration.x; e->guidanceDemand[1]=output.acceleration.y; e->guidanceDemand[2]=output.acceleration.z;
        if (output.active && !heat && spec.seeker!="semi_active_radar" &&
                (old.phase==ms::WeaponPhase::Midcourse || old.phase==ms::WeaponPhase::Unguided)) recordEngagement("activation",*e);
        if (old.tracking && !output.tracking) recordEngagement("track_loss",*e,old.holding);
        if (!old.tracking && output.tracking) recordEngagement("acquisition",*e,output.holding);
        if (output.holding!=old.holding && output.holding>=ms::kDecoyIds && output.holding<kDesignations)
            recordEngagement("decoy_capture",*e,output.holding);
    }
    // Observe t0 first, then advance each decoy in its own local environment.
    // The owned trajectories retain t0 and the final partial live interval even
    // after expiry or removal of the dispensing aircraft.
    const double direction=environment_.windDirectionDeg*ms::geom::kPi/180;
    const double north=-environment_.windSpeedMs*std::cos(direction);
    const double east=-environment_.windSpeedMs*std::sin(direction);
    for (auto* dispenser : dispensers()) {
        auto trajectories=dispenser->advance(dt,[&](const auto& d) {
            const auto geo=terminal({d.position.x,d.position.y,d.position.z});
            const double sl=std::sin(geo.latitudeRad),cl=std::cos(geo.latitudeRad);
            const double so=std::sin(geo.longitudeRad),co=std::cos(geo.longitudeRad);
            return ms::countermeasure::Motion{{-cl*co,-cl*so,-sl},
                {-north*sl*co-east*so,-north*sl*so+east*co,north*cl}};
        });
        decoyTrajectories_.insert(decoyTrajectories_.end(),trajectories.begin(),trajectories.end());
    }
    retiredDispensers_.erase(std::remove_if(retiredDispensers_.begin(),retiredDispensers_.end(),
        [](const auto& d){return d->decoys().empty();}),retiredDispensers_.end());
}

sim::VehicleState World::engagementStateAt(const Entry& e,double offsetS) const {
    const auto& path=e.engagementPath;
    const auto& states=e.engagementStates;
    if (!std::isfinite(offsetS) || offsetS<0 || states.size()!=path.size() || !ms::weapon::validMotionPath(path))
        return pool_->states()[e.slot];
    for (std::size_t k=0;k<path.size();++k)
        if (offsetS==path[k].offsetS) return states[k];
    for (std::size_t k=1;k<path.size();++k) if (offsetS<=path[k].offsetS) {
        const double f=std::clamp((offsetS-path[k-1].offsetS)/(path[k].offsetS-path[k-1].offsetS),0.0,1.0);
        auto state=states[k];
        const auto pos=ms::weapon::motionAt(path,offsetS);
        state.positionEcef[0]=pos.x; state.positionEcef[1]=pos.y; state.positionEcef[2]=pos.z;
        core::attitude::slerp(states[k-1].attitudeEcefToBody,states[k].attitudeEcefToBody,f,state.attitudeEcefToBody);
        core::attitude::bodyToEcefFromQuaternion(state.attitudeEcefToBody,state.rotationBodyToEcef);
        const auto a=velocity(states[k-1]),b=velocity(states[k]);
        const double v[3]={a.x+f*(b.x-a.x),a.y+f*(b.y-a.y),a.z+f*(b.z-a.z)};
        ms::geom::toBody(state,v,state.velocityBodyMs);
        for (std::size_t axis=0;axis<3;++axis)
            state.angularRateBodyRadS[axis]=states[k-1].angularRateBodyRadS[axis]+
                f*(states[k].angularRateBodyRadS[axis]-states[k-1].angularRateBodyRadS[axis]);
        const auto geo=terminal(pos);
        state.latitudeRad=geo.latitudeRad; state.longitudeRad=geo.longitudeRad; state.altitudeMslM=geo.heightM;
        const double sl=std::sin(geo.latitudeRad),cl=std::cos(geo.latitudeRad);
        const double so=std::sin(geo.longitudeRad),co=std::cos(geo.longitudeRad);
        state.velocityNedMs[0]=-sl*co*v[0]-sl*so*v[1]+cl*v[2];
        state.velocityNedMs[1]=-so*v[0]+co*v[1];
        state.velocityNedMs[2]=-cl*co*v[0]-cl*so*v[1]-sl*v[2];
        state.simTime=states[k-1].simTime+f*(states[k].simTime-states[k-1].simTime);
        return state;
    }
    return states.back();
}

std::optional<ms::damage::V3> World::engagementCgAt(const Entry& e,double offsetS) const {
    const auto& path=e.engagementPath;
    const auto& cg=e.engagementCg;
    if (!std::isfinite(offsetS) || offsetS<0 ||
        path.size()!=cg.size() || path.size()!=e.engagementStates.size() ||
        !ms::weapon::validMotionPath(path)) return std::nullopt;
    for (std::size_t k=0;k<path.size();++k) if (offsetS==path[k].offsetS) {
        if (k && !e.engagementStates.front().crashed &&
            (e.engagementStates[k].crashed || e.engagementStates[k].diverged)) return std::nullopt;
        return cg[k];
    }
    for (std::size_t k=1;k<path.size();++k) if (offsetS<=path[k].offsetS) {
        if (!e.engagementStates.front().crashed &&
            (e.engagementStates[k].crashed || e.engagementStates[k].diverged)) return std::nullopt;
        if (!cg[k-1] || !cg[k]) return std::nullopt;
        const double f=std::clamp((offsetS-path[k-1].offsetS)/(path[k].offsetS-path[k-1].offsetS),0.0,1.0);
        const auto a=*cg[k-1],b=*cg[k];
        const ms::damage::V3 value{std::fma(f,b.x-a.x,a.x),std::fma(f,b.y-a.y,a.y),std::fma(f,b.z-a.z,a.z)};
        if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z)) return std::nullopt;
        return value;
    }
    // The native stopped-body convention is a constant final pose/CG, never
    // a prediction of continuing flight after the captured history ends.
    if (offsetS<=path.back().offsetS ||
        (e.engagementStates.front().crashed && e.engagementStates.back().crashed)) return cg.back();
    return std::nullopt;
}

std::optional<ms::weapon::MaterialPointSample> World::weaponPointAt(const Entry& e,double offsetS,
    const std::optional<ms::damage::V3>& pointFromOrigin) const {
    const auto state=engagementStateAt(e,offsetS);
    if (!pointFromOrigin) {
        ms::weapon::MaterialPointSample out;
        out.valid=true; out.position={state.positionEcef[0],state.positionEcef[1],state.positionEcef[2]};
        double v[3]; ms::geom::velocityEcef(state,v); out.velocity={v[0],v[1],v[2]};
        const auto* r=state.rotationBodyToEcef; out.axis={r[0],r[3],r[6]};
        const auto* w=state.angularRateBodyRadS;
        out.angularVelocity={r[0]*w[0]+r[1]*w[1]+r[2]*w[2],r[3]*w[0]+r[4]*w[1]+r[5]*w[2],r[6]*w[0]+r[7]*w[1]+r[8]*w[2]};
        return out;
    }
    const auto cg=engagementCgAt(e,offsetS);
    if (!cg) return std::nullopt;
    auto out=ms::weapon::materialPoint(state,*cg,*pointFromOrigin);
    if (!out.valid) return std::nullopt;
    return out;
}

bool World::groundWeaponContact(Entry& e,const sim::TerrainContactSnapshotV1& observed,double age,double offset) {
    if(!e.terrainContactEnabled || !e.fuzeAdapter || !e.warheadAdapter || e.origin.jettisoned)return false;
    const auto& spec=e.fuzeAdapter->spec();
    const double dt=options_.dt*options_.frameSkip;
    ms::weapon::ContactFuzeResult contact;
    contact.valid=true;contact.contact=true;contact.triggered=ms::weapon::armed(age,spec) && age<spec.lifeS;
    contact.offsetS=offset;contact.fraction=offset/dt;
    // The signal is the observed native terrain strike, not a predicted hit.
    if(!e.fuzeAdapter->commitContact(contact))return false;
    const auto* cg=observed.cgFromOriginBodyM;
    if(!std::all_of(cg,cg+3,[](double x){return std::isfinite(x);}) ||
       (e.fixedBurstRequired && !e.burstFromOriginBodyM))return false;
    auto point=ms::weapon::materialPoint(observed.state,{cg[0],cg[1],cg[2]},
        e.burstFromOriginBodyM.value_or(ms::damage::V3{cg[0],cg[1],cg[2]}));
    if(!point.valid)return false;
    e.fuzeAdapter->cancelDelay();
    if(spec.fuzeModelVersion==1 && spec.fuzeDelayS>0) {
        const double due=age+spec.fuzeDelayS;
        if(!std::isfinite(due) || due>=spec.lifeS)return false;
        e.groundBurstAgeS=due;
    } else {e.arrestedBurst=point;detonateArrestedWeapon(e,contact.fraction);}
    return true;
}

void World::detonateArrestedWeapon(Entry& e,double fraction) {
    if(!e.arrestedBurst || e.weaponEnded)return;
    const auto point=*e.arrestedBurst;
    Detonation det;det.store=e.origin.store;det.weapon=e.info.id;det.stepFraction=fraction;
    for(int k=0;k<3;++k) {
        det.positionEcef[k]=(&point.position.x)[k];
        det.velocityEcef[k]=(&point.velocity.x)[k];
        det.axisEcef[k]=(&point.axis.x)[k];
    }
    const auto damage=detonateAt(det,nullptr,dynamic_cast<ms::damage::WarheadSceneExecutionV2*>(e.warheadAdapter.get())!=nullptr);
    recordEngagement("detonation",e,0,"ground_contact");
    auto& event=combatEvents_.back();
    std::copy_n(det.positionEcef,3,event.positionEcef);
    std::copy_n(det.velocityEcef,3,event.velocityEcef);
    std::set<std::uint32_t> victims;
    for(const auto& hit:damage)if(hit.kind!=static_cast<std::uint8_t>(ipc::EngagementKind::Detonation) && hit.slot<entries_.size()) {
        const auto* victim=entries_[hit.slot].get();
        if(victim && victims.insert(victim->info.id).second)recordEngagement("damage",e,victim->info.id,"warhead",victim->crewLost);
    }
    auto ended=pool_->states()[e.slot];
    std::copy_n(det.positionEcef,3,ended.positionEcef);
    ms::geom::toBody(ended,det.velocityEcef,ended.velocityBodyMs);
    const auto location=terminal({point.position.x,point.position.y,point.position.z});
    ended.latitudeRad=location.latitudeRad;ended.longitudeRad=location.longitudeRad;ended.altitudeMslM=location.heightM;
    const double sl=std::sin(location.latitudeRad),cl=std::cos(location.latitudeRad);
    const double so=std::sin(location.longitudeRad),co=std::cos(location.longitudeRad);
    const auto* v=det.velocityEcef;
    ended.velocityNedMs[0]=-sl*co*v[0]-sl*so*v[1]+cl*v[2];
    ended.velocityNedMs[1]=-so*v[0]+co*v[1];ended.velocityNedMs[2]=-cl*co*v[0]-cl*so*v[1]-sl*v[2];
    endWeapon(e,ms::WeaponEnd::Detonated,ended);
}

void World::finishEngagements() {
    // All offsets use observed native integration ordinals. A terminal pose
    // may have been corrected by the FDM and cannot define a moving segment.
    const auto capturedCutoff=[&](const Entry& body) {
        const auto& path=body.engagementPath;
        const auto& states=body.engagementStates;
        if (path.empty() || states.size()!=path.size()) return std::pair<double,bool>{0,true};
        double end=std::min(options_.dt*options_.frameSkip,path.back().offsetS);
        const bool terminal=states.back().crashed || states.back().diverged;
        if (terminal) end=path.size()>1 ? path[path.size()-2].offsetS : 0;
        if (states.back().crashed) {
            if (const auto* source=dynamic_cast<const sim::TerrainStrikeSourceV1*>(&pool_->vehicle(body.slot))) {
                sim::VehicleState incoming;
                if (source->terrainStrikeStateV1(incoming) && std::isfinite(incoming.simTime)) {
                    // The incoming state was captured at an actual native
                    // boundary. Resolve that raw epoch, never subtract clocks
                    // or round it forward into an unobserved integration.
                    double nativeEnd=0;
                    bool known=incoming.simTime<=states.front().simTime;
                    for (std::size_t k=0;!known && k<states.size();++k)
                        if (states[k].simTime==incoming.simTime) {nativeEnd=path[k].offsetS;known=true;}
                    end=known ? std::min(end,nativeEnd) : 0;
                }
            }
        }
        return std::pair<double,bool>{end,terminal};
    };
    const auto capturedPrefix=[](std::vector<ms::weapon::MotionSample> path,double end) {
        while (!path.empty() && path.back().offsetS>end) path.pop_back();
        return path;
    };
    std::vector<std::uint32_t> ended;
    std::vector<std::uint32_t> weapons;
    for (const auto& e : entries_) if (e && (e->guidance || e->fuzeAdapter || e->contactNoseFromOriginBodyM) && !e->weaponEnded) weapons.push_back(e->info.id);
    for (const auto id : weapons) if (Entry* e=entry(id)) {
        if(e->arrestedBurst) {
            const double dt=options_.dt*options_.frameSkip;
            const double before=e->fuzeAgeBoundaryS;e->fuzeAgeBoundaryS+=dt;
            if(e->groundBurstAgeS && *e->groundBurstAgeS>=before && *e->groundBurstAgeS<=e->fuzeAgeBoundaryS)
                detonateArrestedWeapon(*e,(*e->groundBurstAgeS-before)/dt);
            else if(!e->groundBurstAgeS || *e->groundBurstAgeS<before || e->fuzeAgeBoundaryS>=e->fuzeAdapter->spec().lifeS)
                endWeapon(*e,ms::WeaponEnd::Expired,pool_->states()[e->slot]);
            if(e->weaponEnded)ended.push_back(e->info.id);
            continue;
        }
        auto spec=e->fuzeAdapter ? e->fuzeAdapter->spec() : e->guidance ? e->guidance->spec() : ms::weapon::Spec{};
        if (!e->guidance && !e->fuzeAdapter) { spec.armingS=0; spec.lifeS=std::numeric_limits<double>::max(); }
        bool geometricSceneV0=false;
        if(spec.fuzeModelVersion==0&&e->payloadProfile) {
            if(const auto* d=e->payloadProfile->find("fuze")) {
                const auto policy=d->bindings.find("target_detection_v1");
                geometricSceneV0=policy!=d->bindings.end()&&policy->second=="native.time_armed_geometric_scene.v1";
            }
        }
        const double age=simTime_-e->info.launchTimeS;
        const auto s=pool_->states()[e->slot];
        e->origin.burning=totalWeaponThrust(s)>.1;
        const double dt=options_.dt*options_.frameSkip;
        const double ageBefore=e->fuzeAgeBoundaryS;
        e->fuzeAgeBoundaryS=ageBefore+dt;
        sim::TerrainContactSnapshotV1 groundContact;
        std::optional<double> groundOffset;
        if(e->terrainContactEnabled && e->fuzeAdapter && !e->fuzeAdapter->contactState().consumed()) {
            if(const auto* source=dynamic_cast<const sim::TerrainContactSourceV1*>(&pool_->vehicle(e->slot))) {
                if(source->terrainContactSnapshotV1(groundContact) && !e->engagementStates.empty()) {
                    // Integer native outer/substep ordinals survive accumulated
                    // floating-clock rounding at the final capture boundary.
                    groundOffset=sim::terrainContactOffsetV1(groundContact,e->engagementStates.front().stepCount,
                        static_cast<unsigned>(options_.frameSkip),options_.dt);
                }
            }
        }
        const auto pendingGround=[&] {
          if(e->groundBurstAgeS && *e->groundBurstAgeS<=e->fuzeAgeBoundaryS) {
            const double offset=*e->groundBurstAgeS-ageBefore;
            auto point=offset>=0 ? weaponPointAt(*e,offset,e->burstFromOriginBodyM) : std::nullopt;
            if(!point && s.crashed) {
                double cg[3];
                if(pool_->vehicle(e->slot).cgFromOriginBodyM(cg))point=ms::weapon::materialPoint(s,
                    {cg[0],cg[1],cg[2]},e->burstFromOriginBodyM.value_or(ms::damage::V3{cg[0],cg[1],cg[2]}));
            }
            if(point && point->valid){e->arrestedBurst=*point;detonateArrestedWeapon(*e,std::clamp(offset/dt,0.0,1.0));}
            else endWeapon(*e,ms::WeaponEnd::Expired,s);
            ended.push_back(e->info.id);return true;
          }
          return e->groundBurstAgeS.has_value();
        };
        if(pendingGround())continue;
        const auto ownCutoff=capturedCutoff(*e);
        const double liveEnd=ownCutoff.first;
        bool burst=false;
        double fraction=1;
        std::uint32_t burstTarget=e->origin.holding;
        bool contactInitiation=false;
        std::optional<ContactTransfer> mechanical;
        auto sensorPath=capturedPrefix(e->engagementPath,liveEnd);
        if (spec.fuzeModelVersion==1) {
            if (!e->fuzeFromOriginBodyM) sensorPath.clear();
            else for (auto& sample : sensorPath) {
                const auto sensor=weaponPointAt(*e,sample.offsetS,e->fuzeFromOriginBodyM);
                if (!sensor) {sensorPath.clear();break;}
                sample.position={sensor->position.x,sensor->position.y,sensor->position.z};
            }
        }
        std::optional<std::pair<double,std::uint32_t>> firstProximity;
        const auto evaluateFuze=[&](const std::vector<ms::weapon::MotionSample>& targetPath,double end,bool exclusive,
                                    std::uint32_t targetId,ms::weapon::FuzeTurnStateV1* history=nullptr) {
            const auto evaluate=[&] {
                return e->fuzeAdapter
                    ? e->fuzeAdapter->evaluateProximity(sensorPath,targetPath,
                        spec.fuzeModelVersion==1 ? ageBefore : std::max(0.0,age-dt),dt,end,exclusive,targetId)
                    : spec.fuzeModelVersion==1
                    ? e->fuzeTurn.evaluate(sensorPath,targetPath,ageBefore,dt,spec,end,exclusive,targetId)
                    : ms::weapon::sweptPathFuze(sensorPath,targetPath,std::max(0.0,age-dt),dt,spec,end,exclusive);
            };
            ms::weapon::FuzeResult fuze;
            if(history) {
                // Reuse the selected component without changing its interface or
                // mixing distinct objects' boundary-CPA histories.
                auto& active=e->fuzeAdapter ? e->fuzeAdapter->turnState() : e->fuzeTurn;
                std::swap(active,*history);
                struct Restore {
                    ms::weapon::FuzeTurnStateV1& active;
                    ms::weapon::FuzeTurnStateV1& history;
                    ~Restore(){std::swap(active,history);}
                } restore{active,*history};
                fuze=evaluate();
            } else fuze=evaluate();
            if (fuze.valid && std::isfinite(fuze.missM)) {
                e->origin.missM=std::isfinite(e->origin.missM)?std::min(e->origin.missM,fuze.missM):fuze.missM;
                if (spec.fuzeModelVersion==1||geometricSceneV0) {
                    if (fuze.triggered) {
                        const auto candidate=std::make_pair(fuze.fraction,targetId);
                        if(!firstProximity||candidate<*firstProximity)firstProximity=candidate;
                    }
                } else {burst=fuze.triggered;fraction=fuze.fraction;}
            }
        };
        if((spec.fuzeModelVersion==1||geometricSceneV0)&&!e->origin.jettisoned) {
            if(geometricSceneV0)e->proximityTurnsV1.clear();
            std::set<std::pair<std::uint32_t,std::uint64_t>> present;
            // This explicitly selected ideal geometric sensor is time-armed.
            // Its physical scene is independent of main-seeker detections and
            // referee attribution; no scene data enter guidance here. V0
            // opt-in profiles retain their existing constrained-minimum law;
            // V1 retains its authored sensor station and per-target turn state.
            for(const auto& candidate:entries_)if(candidate&&candidate->info.id!=e->info.id&&
                (candidate->info.role==ms::Role::Aircraft||candidate->info.role==ms::Role::Debris)&&
                ms::weapon::validMotionPath(candidate->engagementPath)&&
                candidate->engagementStates.size()==candidate->engagementPath.size()) {
                const auto key=std::make_pair(candidate->info.id,candidate->info.generation);present.insert(key);
                const auto targetCutoff=capturedCutoff(*candidate);
                evaluateFuze(capturedPrefix(candidate->engagementPath,targetCutoff.first),std::min(liveEnd,targetCutoff.first),
                    ownCutoff.second||(targetCutoff.second&&targetCutoff.first<=liveEnd),candidate->info.id,
                    spec.fuzeModelVersion==1?&e->proximityTurnsV1[key]:nullptr);
            }
            // Existing finite decoy bodies remain physical candidates in this
            // ideal sphere. It claims no RF/optical fuze discrimination law.
            for(const auto& trajectory:decoyTrajectories_)if(trajectory.valid) {
                const auto decoyId=trajectory.before.state.id;
                const auto key=std::make_pair(decoyId,std::uint64_t{0});
                const double end=std::min(liveEnd,trajectory.liveDurationS);
                std::vector<ms::weapon::MotionSample> path;
                for(const auto& native:e->engagementPath) {
                    const double offset=std::min(native.offsetS,end);
                    if(!path.empty()&&offset<=path.back().offsetS)continue;
                    ms::countermeasure::Sample sample;
                    if(!ms::countermeasure::sampleAt(trajectory,offset,sample)){path.clear();break;}
                    const auto p=sample.state.position;path.push_back({offset,{p.x,p.y,p.z}});
                }
                if(!ms::weapon::validMotionPath(path))continue;
                present.insert(key);
                evaluateFuze(path,end,ownCutoff.second||(trajectory.expires&&trajectory.liveDurationS<=liveEnd),decoyId,
                    spec.fuzeModelVersion==1?&e->proximityTurnsV1[key]:nullptr);
            }
            for(auto it=e->proximityTurnsV1.begin();it!=e->proximityTurnsV1.end();)
                if(!present.count(it->first))it=e->proximityTurnsV1.erase(it);else ++it;
        } else if (spec.fuzeModelVersion==0&&!geometricSceneV0&&e->origin.tracking) {
            e->proximityTurnsV1.clear();
            const Entry* held=entry(e->origin.holding);
            if (held) {
                const auto targetCutoff=capturedCutoff(*held);
                const double end=std::min(liveEnd,targetCutoff.first);
                evaluateFuze(capturedPrefix(held->engagementPath,targetCutoff.first),end,
                    ownCutoff.second || (targetCutoff.second && targetCutoff.first<=liveEnd),e->origin.holding);
            } else for (const auto& trajectory : decoyTrajectories_)
                if (trajectory.valid && trajectory.before.state.id==e->origin.holding) {
                    const double end=std::min(liveEnd,trajectory.liveDurationS);
                    std::vector<ms::weapon::MotionSample> path;
                    for (const auto& native : e->engagementPath) {
                        const double offset=std::min(native.offsetS,end);
                        if (!path.empty() && offset<=path.back().offsetS) continue;
                        ms::countermeasure::Sample sample;
                        if (!ms::countermeasure::sampleAt(trajectory,offset,sample)) {path.clear();break;}
                        const auto p=sample.state.position;
                        path.push_back({offset,{p.x,p.y,p.z}});
                    }
                    evaluateFuze(path,end,s.crashed || s.diverged || (trajectory.expires && trajectory.liveDurationS<=liveEnd),e->origin.holding);
                }
        } else {
            e->proximityTurnsV1.clear();
            if(e->fuzeAdapter)e->fuzeAdapter->resetTurnHistory();else e->fuzeTurn.reset();
        }
        if(firstProximity) {
            if(spec.fuzeModelVersion==1) {
                const double triggerAge=ageBefore+firstProximity->first*dt;
                if(e->fuzeAdapter)e->fuzeAdapter->scheduleDelay(triggerAge,firstProximity->second);
                else e->fuzeDelay.schedule(triggerAge,firstProximity->second,spec);
            } else {
                // The V0 law has no newly invented delay/latch. Only ownership
                // of its physical candidates and stable earliest choice differ.
                burst=true;fraction=firstProximity->first;burstTarget=firstProximity->second;
            }
        }
        if (spec.fuzeModelVersion==1) {
            const auto due=e->fuzeAdapter ? e->fuzeAdapter->dueDelay(ageBefore,dt,liveEnd,s.crashed || s.diverged)
                : e->fuzeDelay.due(ageBefore,dt,spec,liveEnd,s.crashed || s.diverged);
            if (due.valid && due.due) {burst=true;fraction=due.fraction;burstTarget=due.targetId;}
        }
        const auto& contactState=e->fuzeAdapter ? e->fuzeAdapter->contactState() : e->contactFuze;
        const auto cancelDelay=[&] {
            if (e->fuzeAdapter) e->fuzeAdapter->cancelDelay(); else e->fuzeDelay.cancel();
        };
        if (!contactState.consumed() && e->contactNoseFromOriginBodyM &&
            captureEngagementMotion_ && ms::weapon::validMotionPath(e->engagementPath) &&
            e->engagementPath.size()>=2 && e->engagementStates.size()==e->engagementPath.size() &&
            e->engagementCg.size()==e->engagementPath.size()) {
            double contactEnd=groundOffset ? std::min(liveEnd,*groundOffset) : liveEnd;
            bool exclusive=ownCutoff.second;
            ms::weapon::ContactFuzeResult firstContact;
            std::uint32_t physicalVictim=0;
            // Physical geometry is independent of seeker holding/tracking.
            // Include the launcher: initial overlap is handled by the pure core.
            for (const auto& candidate : entries_) if (candidate && candidate->info.role==ms::Role::Aircraft &&
                    candidate->info.id!=e->info.id && candidate->slot<stepStates_.size() && contactEnd>0) {
                const auto frame=airframes_.find(candidate->info.aircraft);
                if (frame==airframes_.end() || !ms::weapon::validMotionPath(candidate->engagementPath) ||
                    candidate->engagementStates.size()!=candidate->engagementPath.size() ||
                    candidate->engagementCg.size()!=candidate->engagementPath.size()) continue;
                const auto& targetPath=candidate->engagementPath;
                const auto& targetStates=candidate->engagementStates;
                double end=contactEnd;
                bool exclusiveEnd=exclusive;
                if (!targetStates.back().crashed && targetPath.back().offsetS<end) {
                    end=targetPath.back().offsetS;exclusiveEnd=false;
                }
                if (targetStates.back().diverged ||
                    (targetStates.back().crashed && !stepStates_[candidate->slot].crashed)) {
                    const auto targetCutoff=capturedCutoff(*candidate);
                    if (targetCutoff.first<=end) {end=targetCutoff.first;exclusiveEnd=true;}
                }
                if (!(end>0)) continue;
                std::vector<double> times{0,end};
                const std::vector<ms::weapon::MotionSample>* paths[]={&e->engagementPath,&targetPath};
                for (const auto* path : paths)
                    for (const auto& sample : *path) if (sample.offsetS>0 && sample.offsetS<end) times.push_back(sample.offsetS);
                std::sort(times.begin(),times.end());
                times.erase(std::unique(times.begin(),times.end()),times.end());
                std::vector<ms::weapon::ContactSample> relative;
                relative.reserve(times.size());
                for (const double offset : times) {
                    const auto ownCg=engagementCgAt(*e,offset),targetCg=engagementCgAt(*candidate,offset);
                    if (!ownCg || !targetCg) {relative.push_back({offset,{},false});continue;}
                    const auto ownState=engagementStateAt(*e,offset),targetState=engagementStateAt(*candidate,offset);
                    const auto rotate=[](const sim::VehicleState& state,ms::damage::V3 v) {
                        const auto* r=state.rotationBodyToEcef;
                        return ms::damage::V3{r[0]*v.x+r[1]*v.y+r[2]*v.z,r[3]*v.x+r[4]*v.y+r[5]*v.z,
                                              r[6]*v.x+r[7]*v.y+r[8]*v.z};
                    };
                    const auto nose=ms::damage::V3{ownState.positionEcef[0],ownState.positionEcef[1],ownState.positionEcef[2]}+
                        rotate(ownState,*e->contactNoseFromOriginBodyM-*ownCg);
                    const auto origin=ms::damage::V3{targetState.positionEcef[0],targetState.positionEcef[1],targetState.positionEcef[2]}-
                        rotate(targetState,*targetCg);
                    const auto delta=nose-origin;
                    const auto* r=targetState.rotationBodyToEcef;
                    relative.push_back({offset,{r[0]*delta.x+r[3]*delta.y+r[6]*delta.z,
                                                r[1]*delta.x+r[4]*delta.y+r[7]*delta.z,
                                                r[2]*delta.x+r[5]*delta.y+r[8]*delta.z}});
                }
                const auto geometry=candidate->damage ? candidate->damage->attachedGeometry() : ms::damage::AttachedGeometryView(*frame->second);
                const ms::weapon::ContactWindow window{std::max(0.0,age-dt),dt,spec.armingS,spec.lifeS,end,exclusiveEnd};
                const auto contact=e->fuzeAdapter ? e->fuzeAdapter->evaluateContact(relative,geometry,window)
                    : ms::weapon::contactFuze(relative,geometry,window);
                if (contact.valid && contact.contact && (!firstContact.contact || contact.offsetS<firstContact.offsetS ||
                    (contact.offsetS==firstContact.offsetS && candidate->info.id<physicalVictim))) {
                    firstContact=contact;physicalVictim=candidate->info.id;
                }
            }
            // Commit the earliest physical contact even before arming. An
            // earlier proximity initiation retains its existing semantics;
            // exact initiation-time ties also preserve proximity priority.
            const bool committed=e->fuzeAdapter ? e->fuzeAdapter->commitContact(firstContact) : e->contactFuze.commit(firstContact);
            const bool armedContact=committed && (e->guidance || e->fuzeAdapter) && !e->origin.jettisoned;
            if (firstContact.contact && (!burst || firstContact.fraction<=fraction)) {
                mechanical=contactTransfer(*e,physicalVictim,firstContact);
                if (armedContact && (!burst || firstContact.fraction<fraction)) {
                    burst=true;fraction=firstContact.fraction;burstTarget=physicalVictim;contactInitiation=true;
                    cancelDelay();
                } else if (mechanical && burst && firstContact.fraction<fraction) {
                    // An earlier mechanical impulse invalidates a later
                    // captured free-flight encounter; do not detonate from it.
                    burst=false;
                    cancelDelay();
                }
            }
        }
        if(groundOffset && !e->fuzeAdapter->contactState().consumed() &&
           (!burst || *groundOffset<fraction*dt)) {
            groundWeaponContact(*e,groundContact,ageBefore+*groundOffset,*groundOffset);
            if(e->weaponEnded){ended.push_back(e->info.id);continue;}
            if(pendingGround())continue;
        }
        if (burst) {
            Detonation det;
            det.store=e->origin.store; det.weapon=e->info.id;
            det.stepFraction=fraction;
            const auto burstState=engagementStateAt(*e,fraction*dt);
            const auto burstPoint=weaponPointAt(*e,fraction*dt,e->burstFromOriginBodyM);
            if (!burstPoint || (e->fixedBurstRequired && !e->burstFromOriginBodyM)) {
                LOG_WARN("session") << "weapon burst material point unavailable for " << e->info.name;
                continue;
            }
            for(int k=0;k<3;++k) {
                det.positionEcef[k]=(&burstPoint->position.x)[k];
                det.axisEcef[k]=(&burstPoint->axis.x)[k];
                det.velocityEcef[k]=(&burstPoint->velocity.x)[k];
            }
            if (contactInitiation) if (const auto* victim=entry(burstTarget)) {
                const auto at=engagementStateAt(*victim,fraction*dt);
                e->origin.missM=length(delta(point(burstState.positionEcef),point(at.positionEcef)));
            }
            if (mechanical) contactReaction(*e,*mechanical);
            const auto damage=detonateAt(det,mechanical ? &*mechanical : nullptr,dynamic_cast<ms::damage::WarheadSceneExecutionV2*>(e->warheadAdapter.get())!=nullptr);
            recordEngagement("detonation",*e,burstTarget,contactInitiation?"contact":"");
            combatEvents_.back().positionEcef[0]=det.positionEcef[0];
            combatEvents_.back().positionEcef[1]=det.positionEcef[1];
            combatEvents_.back().positionEcef[2]=det.positionEcef[2];
            std::copy_n(det.velocityEcef,3,combatEvents_.back().velocityEcef);
            std::set<std::uint32_t> victims;
            for (const auto& hit : damage) if (hit.kind!=static_cast<std::uint8_t>(ipc::EngagementKind::Detonation) && hit.slot<entries_.size()) {
                const auto* victim=entries_[hit.slot].get();
                if (victim && victims.insert(victim->info.id).second)
                    recordEngagement("damage",*e,victim->info.id,"warhead",victim->crewLost);
            }
            auto endedState=burstState;
            std::copy_n(det.positionEcef,3,endedState.positionEcef);
            ms::geom::toBody(endedState,det.velocityEcef,endedState.velocityBodyMs);
            const auto pose=terminal(point(det.positionEcef));
            endedState.latitudeRad=pose.latitudeRad; endedState.longitudeRad=pose.longitudeRad; endedState.altitudeMslM=pose.heightM;
            const double sl=std::sin(pose.latitudeRad),cl=std::cos(pose.latitudeRad),so=std::sin(pose.longitudeRad),co=std::cos(pose.longitudeRad);
            const auto* v=det.velocityEcef;
            endedState.velocityNedMs[0]=-sl*co*v[0]-sl*so*v[1]+cl*v[2];
            endedState.velocityNedMs[1]=-so*v[0]+co*v[1]; endedState.velocityNedMs[2]=-cl*co*v[0]-cl*so*v[1]-sl*v[2];
            endWeapon(*e,ms::WeaponEnd::Detonated,endedState); ended.push_back(e->info.id);
        } else if (mechanical) {
            contactReaction(*e,*mechanical);
            contactDamage(*e,*mechanical);
        } else if ((e->guidance || e->fuzeAdapter) && !s.crashed && (age>=spec.lifeS || s.diverged)) {
            recordEngagement("expiry",*e,0,s.diverged?"flight_diverged":"battery_expired");
            endWeapon(*e,ms::WeaponEnd::Expired,s); ended.push_back(e->info.id);
        }
    }
    for (const auto id : ended) removeVehicle(id);
}

ms::LaunchZone World::launchZone(std::uint32_t vehicle,const std::string& store,std::uint32_t target) const {
    ms::LaunchZone zone;
    const Entry* e=entry(vehicle); const auto* estimate=vehicleState(target);
    if (!e || !estimate) return zone;
    const ms::StoreCard* card=nullptr;
    for (const auto& s : e->ms.stores()) if (s.store==store) {card=e->ms.storeCard(s.index);break;}
    if (!card || card->storeClass!="aam" || card->model.empty()) return zone;
    const auto& own=pool_->states()[e->slot];
    const V r=delta(point(estimate->positionEcef),point(own.positionEcef));
    const double range=length(r); const V tv=velocity(*estimate);
    const double closing=-dot(tv,r)/std::max(range,1.0);
    // basis: assumed. Advisory energy envelope, not a second trajectory integrator or a hit guarantee.
    const double reference=card->value("launch_zone_reference_m",infrared(card->seeker)?14000.0:60000.0);
    const double altitude=std::clamp(.65+own.altitudeMslM/20000.0,.5,1.4);
    const double speed=std::clamp(own.airspeedTrueMs/300.0,.5,1.5);
    zone.maximumM=std::max(0.0,reference*altitude*speed+closing*25.0);
    zone.noEscapeM=std::clamp(zone.maximumM*.55-length(tv)*12.0,0.0,zone.maximumM);
    return zone;
}
std::uint32_t World::seekerTarget(std::uint32_t vehicle,const std::string& store) const {
    const Entry* e=entry(vehicle);
    if (!e) return 0;
    const ms::StoreCard* card=nullptr;std::uint16_t index=0;
    for (const auto& s : e->ms.stores()) if (s.store==store && ms::weapon::onRailObservationAvailable(s.status)) {card=e->ms.storeCard(s.index);index=s.index;break;}
    if (!card || !infrared(card->seeker)) return 0;
    return onRailSeekerObservation(*e,*card,index);
}

std::uint32_t World::onRailSeekerObservation(const Entry& launcher,const ms::StoreCard& card,std::uint16_t store) const {
    const auto& own=pool_->states()[launcher.slot];
    std::string error; const auto configured=configuredWeaponSpec(card,error);
    if (!configured) return 0;
    const auto profile=payloadProfile(card,error);if(!profile)return 0;
    if(const auto* descriptor=profile->find("seeker");descriptor&&descriptor->bindings.count("measurement_model")) {
        const auto* cached=sampleOnRailSeeker(launcher,card,store);
        if(!cached||!cached->result.tracking||!cached->result.attributionIndex||
           *cached->result.attributionIndex>=cached->attributions.size())return 0;
        return cached->attributions[*cached->result.attributionIndex];
    }
    ms::components::ProviderContextV1 context{card,*profile,*configured,std::nullopt};
    auto provider=profile->explicitProfile() ? payloadProviders_.seeker.create(*profile->find("seeker"),context,error)
        : ms::components::nativeSeekerProviderV1(configured->seekerModel);
    if(!provider)return 0;
    const auto& seeker=provider->config();
    double best=seeker.version==1 ? seeker.launchFovRad : card.value("launch_fov_deg",25.0)*ms::geom::kPi/180.0;
    std::uint32_t found=0;
    for (const auto& candidate : entries_) if (candidate && candidate->info.role==ms::Role::Aircraft && candidate->info.id!=launcher.info.id) {
        const auto& target=pool_->states()[candidate->slot];
        if (target.crashed || target.diverged) continue;
        const V r=delta(point(target.positionEcef),point(own.positionEcef)), tv=velocity(target);
        const double range=length(r);
        if (range<seeker.minimumRangeM) continue;
        double body[3], raw[3]={r.x,r.y,r.z}; ms::geom::toBody(own,raw,body);
        const double off=std::acos(std::clamp(body[0]/range,-1.0,1.0));
        const double aspect=dot(r,tv)/(range*std::max(length(tv),seeker.aspectSpeedFloorMs));
        const double reach=provider->infraredRange(configured->acquireM,aspect,target.afterburner[0]>seeker.afterburnerThreshold);
        if (off>=best || range>reach) continue;
        const auto path=radioPath(launcher.info.id,candidate->info.id,{});
        if (path && path->terrainMasked) continue;
        best=off; found=candidate->info.id;
    }
    return found;
}

std::optional<ms::damage::AttachedGeometryView> World::seekerGeometryV1(const Entry& target) const {
    if(target.damage)return target.damage->attachedGeometry();
    const auto frame=airframes_.find(target.info.aircraft);
    if(frame==airframes_.end()||!frame->second)return std::nullopt;
    // Intact aircraft already have immutable admitted geometry. A sensor read
    // must not depend on, allocate or reseed the lazy damage-state instance.
    return ms::damage::AttachedGeometryView(*frame->second);
}
std::optional<ms::components::ImageExtentV2> World::projectedSeekerExtent(const Entry& target,
    const sim::VehicleState& observer) const {
    using namespace ms::components::measured;
    using ms::components::measured::dot;
    const auto attached=seekerGeometryV1(target);if(!attached)return std::nullopt;
    const auto& state=pool_->states()[target.slot];
    const auto ray=unit(subtract(point(state.positionEcef),point(observer.positionEcef)));
    const V observerDown{observer.rotationBodyToEcef[2],observer.rotationBodyToEcef[5],observer.rotationBodyToEcef[8]};
    auto right=unit(cross(observerDown,ray));if(!(norm(right)>0))return std::nullopt;
    const auto up=unit(cross(ray,right));
    double cg[3];pool_->vehicle(target.slot).cgFromOriginBodyM(cg);
    if(!std::all_of(cg,cg+3,[](double v){return std::isfinite(v);}))return std::nullopt;
    const auto* rotation=state.rotationBodyToEcef;
    double lowX=std::numeric_limits<double>::infinity(),highX=-lowX,lowY=lowX,highY=-lowX;
    const auto project=[&](ms::damage::V3 pointBody) {
        const double b[3]={pointBody.x-cg[0],pointBody.y-cg[1],pointBody.z-cg[2]};
        V at=point(state.positionEcef);
        at.x+=rotation[0]*b[0]+rotation[1]*b[1]+rotation[2]*b[2];
        at.y+=rotation[3]*b[0]+rotation[4]*b[1]+rotation[5]*b[2];
        at.z+=rotation[6]*b[0]+rotation[7]*b[1]+rotation[8]*b[2];
        const auto relative=subtract(at,point(observer.positionEcef));const auto forward=dot(relative,ray);
        if(!(forward>0)||!finite(relative))return;
        const auto x=std::atan2(dot(relative,right),forward),y=std::atan2(dot(relative,up),forward);
        lowX=std::min(lowX,x);highX=std::max(highX,x);lowY=std::min(lowY,y);highY=std::max(highY,y);
    };
    const auto box=[&](ms::damage::V3 centre,ms::damage::V3 half) {
        for(int x:{-1,1})for(int y:{-1,1})for(int z:{-1,1})
            project(centre+ms::damage::V3{x*half.x,y*half.y,z*half.z});
    };
    const auto& frame=attached->airframe();
    for(const auto i:attached->panelIndices()) {
        const auto& p=frame.panels[i];if(p.surface)for(const auto& corner:p.corners)project(corner);
        else box(p.centre,p.half);
    }
    for(const auto i:attached->componentIndices()) {const auto& c=frame.components[i];box(c.centre,c.half);}
    ms::components::ImageExtentV2 extent{highX-lowX,highY-lowY};
    return validExtent(extent)?std::optional<ms::components::ImageExtentV2>{extent}:std::nullopt;
}

const World::Entry::OnRailSeekerV2* World::sampleOnRailSeeker(const Entry& launcher,
    const ms::StoreCard& card,std::uint16_t store) const {
    std::string error;const auto configured=configuredWeaponSpec(card,error);if(!configured)return nullptr;
    const auto profile=payloadProfile(card,error);if(!profile)return nullptr;
    const auto* descriptor=profile->find("seeker");
    if(!descriptor||!descriptor->bindings.count("measurement_model"))return nullptr;
    auto& cached=launcher.onRailSeekers[store];
    if(cached.card!=&card) {
        cached={};cached.card=&card;cached.startedTimeS=simTime_;
        ms::components::ProviderContextV1 context{card,*profile,*configured,std::nullopt};
        cached.provider=payloadProviders_.seeker.create(*descriptor,context,error);
    }
    auto* sensor=dynamic_cast<ms::components::SeekerExecutionV2*>(cached.provider.get());
    if(!sensor)return nullptr;
    if(cached.timeS==simTime_)return &cached;
    const auto& own=pool_->states()[launcher.slot];
    const auto& config=cached.provider->config();ms::components::SeekerFrameV2 frame;
    ms::components::ReceiverConfigurationV2 receiverScratch;
    const auto* receiverEnvelope=physicalReceiver(cached.provider.get(),descriptor,receiverScratch);
    const auto* receiver=receiverEnvelope?&receiverEnvelope->single:nullptr;
    ms::components::ReceiverConfigurationV3 reticleScratch;
    const auto* reticle=reticleReceiver(cached.provider.get(),descriptor,reticleScratch);
    const auto* thermalScenePolicy=reticle?&reticle->scene:receiver?&receiver->scene:nullptr;
    const bool dualThermal=receiverEnvelope&&receiverEnvelope->dual&&receiverEnvelope->secondBandScene;
    const bool physicalThermal=reticle||(receiver&&(receiver->kind==ms::components::PhysicalReceiverKindV1::Image||receiver->kind==ms::components::PhysicalReceiverKindV1::Quadrant));
    auto thermalScene=physicalThermal?thermalBackground(*thermalScenePolicy,own,simTime_):ms::components::ThermalSceneFrameV1{};
    auto secondThermalScene=dualThermal?thermalBackground(*receiverEnvelope->secondBandScene,own,simTime_):ms::components::ThermalSceneFrameV1{};
    const bool finiteSensor=config.engineering.version==3;
    frame.receiverSeed=Rng::forVehicle(options_.seed,0x4F4E5241494Cull,(static_cast<std::uint64_t>(launcher.info.id)<<16)|store).next();
    frame.bodyAxes=std::array<V,3>{{{own.rotationBodyToEcef[0],own.rotationBodyToEcef[3],own.rotationBodyToEcef[6]},
        {own.rotationBodyToEcef[1],own.rotationBodyToEcef[4],own.rotationBodyToEcef[7]},
        {own.rotationBodyToEcef[2],own.rotationBodyToEcef[5],own.rotationBodyToEcef[8]}}};
    frame.timeS=simTime_;frame.ageS=std::max(0.,simTime_-cached.startedTimeS);frame.dtS=options_.dt*options_.frameSkip;
    frame.position=point(own.positionEcef);frame.velocity=velocity(own);
    frame.boresight={own.rotationBodyToEcef[0],own.rotationBodyToEcef[3],own.rotationBodyToEcef[6]};
    frame.up={std::cos(own.latitudeRad)*std::cos(own.longitudeRad),
              std::cos(own.latitudeRad)*std::sin(own.longitudeRad),std::sin(own.latitudeRad)};
    cached.attributions.clear();
    for(const auto& candidate:entries_)if(candidate&&candidate->info.role==ms::Role::Aircraft&&candidate.get()!=&launcher) {
        const auto& target=pool_->states()[candidate->slot];if(target.crashed||target.diverged)continue;
        const auto relative=delta(point(target.positionEcef),frame.position);const auto range=length(relative);
        if(range<config.minimumRangeM)continue;
        const auto ray=ms::components::measured::unit(relative);
        if(dot(ray,frame.boresight)<=std::cos(config.launchFovRad))continue;
        const auto tv=velocity(target);
        const V targetForward{target.rotationBodyToEcef[0],target.rotationBodyToEcef[3],target.rotationBodyToEcef[6]};
        const auto aspect=finiteSensor?ms::components::EngineeringSeekerV3::sourceAspectCosineV3(relative,targetForward):dot(relative,tv)/(range*std::max(length(tv),config.aspectSpeedFloorMs));
        const auto reach=cached.provider->infraredRange(configured->acquireM,aspect,target.afterburner[0]>config.afterburnerThreshold);
        if(!finiteSensor&&range>reach)continue;
        if(physicalThermal){if(!opticalClear(frame.position,point(target.positionEcef),ground_.get()))continue;}
        else {const auto path=radioPath(launcher.info.id,candidate->info.id,{});if(path&&path->terrainMasked)continue;}
        ms::components::SeekerSampleV2 sample;sample.direction=ray;sample.signal=config.aircraftSignal;
        if(finiteSensor){sample.scene=ms::components::SeekerSceneV3{};sample.scene->rangeM=range;
            if(configured->acquireM>0)sample.signal*=std::pow(reach/configured->acquireM,2);}
        if(configured->seeker=="imaging_infrared")sample.imageExtent=projectedSeekerExtent(*candidate,own);
        if(physicalThermal)if(const auto geometry=seekerGeometryV1(*candidate))if(const auto extent=projectedSeekerExtent(*candidate,own)) {
            appendThermalAircraft(thermalScene,*thermalScenePolicy,own,target,pool_->vehicle(candidate->slot),*geometry,*extent,frame.samples.size());
            if(dualThermal)appendThermalAircraft(secondThermalScene,*receiverEnvelope->secondBandScene,own,target,pool_->vehicle(candidate->slot),*geometry,*extent,frame.samples.size());
        }
        frame.samples.push_back(sample);cached.attributions.push_back(candidate->info.id);
    }
    for(const auto* dispenser:dispensers())for(const auto& decoy:dispenser->decoys()) {
        if(decoy.kind!=ms::countermeasure::Kind::Flare)continue;
        const V at{decoy.position.x,decoy.position.y,decoy.position.z};
        const auto relative=delta(at,frame.position);const auto range=length(relative);
        if(range<config.minimumRangeM||(!finiteSensor&&range>configured->acquireM))continue;
        const auto ray=ms::components::measured::unit(relative);
        if(dot(ray,frame.boresight)<=std::cos(config.launchFovRad))continue;
        ms::rf::PathOptions path;path.ground=ground_.get();
        if(physicalThermal){if(!opticalClear(frame.position,at,ground_.get()))continue;}
        else if(ms::rf::computePath({own.latitudeRad,own.longitudeRad,own.altitudeMslM},terminal(at),path).terrainMasked)continue;
        ms::components::SeekerSampleV2 sample;sample.direction=ray;
        sample.signal=cached.provider->decoySignal(std::max(0.,simTime_-decoy.launchTimeS),true);
        if(finiteSensor){sample.scene=ms::components::SeekerSceneV3{};sample.scene->rangeM=range;}
        if(physicalThermal) {
            const double age=std::max(0.,simTime_-decoy.launchTimeS);
            const auto rate=directionRate(relative,delta(V{decoy.velocity.x,decoy.velocity.y,decoy.velocity.z},frame.velocity));
            appendThermalFlare(thermalScene,*thermalScenePolicy,ray,rate,range,age,frame.samples.size());
            if(dualThermal)appendThermalFlare(secondThermalScene,*receiverEnvelope->secondBandScene,ray,rate,range,age,frame.samples.size());
        }
        // Cartridge flame geometry/signatures are not invented. An absent
        // extent is unavailable imagery, never a semantic flare discriminator.
        frame.samples.push_back(sample);cached.attributions.push_back(decoy.id);
    }
    cached.sceneAttributionsV3[simTime_]=std::move(cached.attributions);
    if(physicalThermal) {
        ms::components::SeekerNavigationPoseV1 aligned;aligned.valid=true;aligned.timeS=simTime_;
        aligned.position=frame.position;aligned.velocity=frame.velocity;aligned.bodyToEcef=*frame.bodyAxes;
        thermalScene.navigation=aligned;
        if(reticle) {
            if(auto* detector=dynamic_cast<ms::components::ThermalReticleSeekerExecutionV1*>(cached.provider.get()))
                cached.result=detector->sampleThermalReticleV1(frame,thermalScene);
            else cached.result={};
        } else if(dualThermal) {
            secondThermalScene.navigation=aligned;
            if(auto* detector=dynamic_cast<ms::components::ThermalDualBandSeekerExecutionV1*>(cached.provider.get()))
                cached.result=detector->sampleThermalDualBandV1(frame,{{thermalScene,secondThermalScene}});
            else cached.result={};
        } else if(auto* detector=dynamic_cast<ms::components::ThermalSeekerExecutionV1*>(cached.provider.get()))cached.result=detector->sampleThermalV1(frame,thermalScene);
        else cached.result={};
    } else if(!descriptor->bindings.count("receiver_model"))cached.result=sensor->sample(frame);
    else cached.result={};
    const auto original=cached.sceneAttributionsV3.find(cached.result.measurementTimeS.value_or(simTime_));
    cached.attributions=original==cached.sceneAttributionsV3.end()?std::vector<std::uint32_t>{}:original->second;
    const double retention=std::max(3.,config.engineering.processingDelayS+config.engineering.memoryS+2*config.engineering.samplePeriodS);
    while(!cached.sceneAttributionsV3.empty()&&(cached.sceneAttributionsV3.begin()->first<simTime_-retention||cached.sceneAttributionsV3.size()>2048))
        cached.sceneAttributionsV3.erase(cached.sceneAttributionsV3.begin());
    cached.timeS=simTime_;return &cached;
}

void World::sampleOnRailSeekers() {
    for(const auto& launcher:entries_)if(launcher) {
        std::set<std::uint16_t> installed;
        for(const auto& store:launcher->ms.stores()) {
            const bool observable=ms::weapon::onRailObservationAvailable(store.status);
            const bool attached=observable||store.status==ms::StoreStatus::PoweringUp||
                store.status==ms::StoreStatus::Releasing||store.status==ms::StoreStatus::PoweringDown;
            if(!attached)continue;
            const auto* card=launcher->ms.storeCard(store.index);if(!card||!infrared(card->seeker))continue;
            if(!observable) {
                std::string error;const auto profile=payloadProfile(*card,error);
                const auto* descriptor=profile?profile->find("seeker"):nullptr;
                if(!descriptor||!descriptor->bindings.count("receiver_model"))continue;
            }
            // Read availability is not component lifetime or processing time.
            // Keep the powered receiver's optical/servo/processing clocks
            // running while preparation gates its public on-rail query.
            installed.insert(store.index);
            sampleOnRailSeeker(*launcher,*card,store.index);
        }
        for(auto it=launcher->onRailSeekers.begin();it!=launcher->onRailSeekers.end();)
            if(!installed.count(it->first))it=launcher->onRailSeekers.erase(it);else ++it;
    }
}
} // namespace fsim::session
