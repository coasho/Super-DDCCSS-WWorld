// Native engagement lifecycle. JSBSim alone integrates weapon motion.
#include "simulation/runtime/world/World.h"
#include "foundation/io/AssetResolver.h"
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
ms::rf::Terminal terminal(V p) {
    const double horizontal=std::hypot(p.x,p.y);
    double latitude=std::atan2(p.z,horizontal*(1-.00669437999014)), n=6378137;
    for(int k=0;k<5;++k) {
        n=6378137/std::sqrt(1-.00669437999014*std::sin(latitude)*std::sin(latitude));
        latitude=std::atan2(p.z+.00669437999014*n*std::sin(latitude),horizontal);
    }
    return {latitude,std::atan2(p.y,p.x),horizontal/std::max(std::cos(latitude),1e-9)-n};
}
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
        const auto& s=pool_->states()[e->slot];
        const auto spec=e->guidance->spec();
        const bool heat=infrared(spec.seeker);
        ms::weapon::Input in;
        in.position=point(s.positionEcef); in.velocity=velocity(s); in.dtS=dt;
        in.boresight=V{s.rotationBodyToEcef[0],s.rotationBodyToEcef[3],s.rotationBodyToEcef[6]};
        in.ageS=simTime_-e->info.launchTimeS;
        in.launchCue=heat?e->launchSeekerCue:0;
        in.up={std::cos(s.latitudeRad)*std::cos(s.longitudeRad),std::cos(s.latitudeRad)*std::sin(s.longitudeRad),std::sin(s.latitudeRad)};
        const auto* d=designation(e->origin.target);
        const Entry* launcher=entry(e->info.launcher);
        const ms::radar::Radar* illuminator=nullptr;
        sim::VehicleState supportedEstimate{};
        if (d && launcher && resolveWeaponTarget(e->info.launcher,e->origin.target,supportedEstimate)) {
            in.supported=true;
            in.estimate=ms::weapon::Observation{d->id,point(supportedEstimate.positionEcef),velocity(supportedEstimate),false};
            const auto r=launcher->ms.radarStatus(d->payload);
            in.illuminated=d->source==comm::kNoSystem && r.illuminating && r.designatedTrack==d->track;
            if (in.illuminated) illuminator=launcher->ms.radar(d->payload);
        }
        const bool semi=spec.seeker=="semi_active_radar";
        const auto illuminatedCandidate=[&](V position) {
            if (!semi) return true;
            if (!illuminator || !in.illuminated || !launcher || !illuminator->illuminating()
                || illuminator->designated()!=d->track) return false;
            const auto& transmitter=pool_->states()[launcher->slot];
            const V ray=delta(position,point(transmitter.positionEcef));
            const double raw[3]={ray.x,ray.y,ray.z}; double body[3]; ms::geom::toBody(transmitter,raw,body);
            if (!ms::weapon::withinIlluminationBeam(body,illuminator->beamAzimuthRad(),illuminator->beamElevationRad(),
                    illuminator->pattern().beamwidthRad(false),illuminator->pattern().beamwidthRad(true))) return false;
            ms::rf::PathOptions path; path.ground=ground_.get(); path.frequencyHz=illuminator->spec().frequencyHz;
            if (!std::isfinite(path.frequencyHz) || path.frequencyHz<=0) return false;
            return !ms::rf::computePath({transmitter.latitudeRad,transmitter.longitudeRad,transmitter.altitudeMslM},terminal(position),path).terrainMasked;
        };
        // Seeker observations, not omniscient updates: range, aspect, gimbal and Doppler gates.
        // basis: assumed. IR aspect/afterburner factors, 28 m/s notch and decoy response are game estimates.
        for (const auto& candidate : entries_) if (candidate && candidate->info.role==ms::Role::Aircraft &&
                candidate->info.id!=e->info.launcher && candidate->info.id!=e->info.id) {
            const auto& target=pool_->states()[candidate->slot];
            if (target.crashed || target.diverged) continue;
            if (!illuminatedCandidate(point(target.positionEcef))) continue;
            const V r=delta(point(target.positionEcef),in.position);
            const double range=length(r);
            if (range<1.0 || (!heat && range>spec.acquireM)) continue;
            const V tv=velocity(target);
            double acquire=spec.acquireM;
            if (heat) {
                const double aspect=dot(r,tv)/(range*std::max(length(tv),1.0));
                acquire=ms::weapon::infraredAcquisitionRangeM(spec.acquireM,aspect,target.afterburner[0]>.5);
            } else {
                const auto cross=radarCrossSection(e->info.id,candidate->info.id,{});
                if (cross && cross->modelled) acquire*=std::clamp(std::pow(std::max(cross->sigmaM2,.001)/5.0,.25),.4,1.0);
                if (dot(r,in.up)/range<-.03 && std::fabs(dot(tv,r)/range)<28.0) continue;
            }
            if (range>acquire) continue;
            // Terrain masking and the curved radio horizon also apply to the seeker's own sensor.
            ms::RadioPathOptions path;
            if (semi && illuminator) path.frequencyHz=illuminator->spec().frequencyHz;
            const auto propagation=radioPath(e->info.id,candidate->info.id,path);
            if (propagation && propagation->terrainMasked) continue;
            in.visible.push_back({candidate->info.id,point(target.positionEcef),tv,false,1.0,acquire});
        }
        for (const auto* dispenser : dispensers()) for (const auto& decoy : dispenser->decoys()) {
            if (heat != (decoy.kind==ms::countermeasure::Kind::Flare)) continue;
            const V p{decoy.position.x,decoy.position.y,decoy.position.z}, v{decoy.velocity.x,decoy.velocity.y,decoy.velocity.z};
            if (!illuminatedCandidate(p)) continue;
            const V r=delta(p,in.position); const double range=length(r);
            if (range<1 || range>spec.acquireM || (!heat && dot(r,in.up)/range<-.03 && std::fabs(dot(v,r)/range)<28)) continue;
            ms::rf::PathOptions path; path.ground=ground_.get();
            if (semi && illuminator) path.frequencyHz=illuminator->spec().frequencyHz;
            const auto propagation=ms::rf::computePath({s.latitudeRad,s.longitudeRad,s.altitudeMslM},terminal(p),path);
            if (propagation.terrainMasked) continue;
            // basis: assumed. Relative signature bloom/fade for deterministic seeker discrimination.
            const double age=std::max(0.0,simTime_-decoy.launchTimeS);
            const double signal=heat?6*std::exp(-age/2.0):3*std::min(1.0,age/.3)*std::exp(-age/6.0);
            if (signal>0) in.visible.push_back({decoy.id,p,v,true,signal});
        }
        const auto old=e->origin;
        const auto output=e->guidance->update(in);
        e->origin.phase=output.phase; e->origin.holding=output.holding;
        e->origin.tracking=output.tracking; e->origin.supported=output.supported;
        e->origin.timeToGoS=output.timeToGoS; e->origin.timeToActiveS=output.timeToActiveS;
        e->origin.burning=s.thrustN[0]>.1;
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
    if (!captureEngagementMotion_ || states.size()!=path.size() || !ms::weapon::validMotionPath(path))
        return pool_->states()[e.slot];
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
    if (!captureEngagementMotion_ || !std::isfinite(offsetS) || offsetS<0 ||
        path.size()!=cg.size() || path.size()!=e.engagementStates.size() ||
        !ms::weapon::validMotionPath(path)) return std::nullopt;
    for (std::size_t k=1;k<path.size();++k) if (offsetS<=path[k].offsetS) {
        if (!cg[k-1] || !cg[k]) return std::nullopt;
        const double f=std::clamp((offsetS-path[k-1].offsetS)/(path[k].offsetS-path[k-1].offsetS),0.0,1.0);
        const auto a=*cg[k-1],b=*cg[k];
        const ms::damage::V3 value{std::fma(f,b.x-a.x,a.x),std::fma(f,b.y-a.y,a.y),std::fma(f,b.z-a.z,a.z)};
        if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z)) return std::nullopt;
        return value;
    }
    // The native stopped-body convention is a constant final pose/CG, never
    // a prediction of continuing flight after the captured history ends.
    if (offsetS<=path.back().offsetS || e.engagementStates.back().crashed) return cg.back();
    return std::nullopt;
}

void World::finishEngagements() {
    std::vector<std::uint32_t> ended;
    std::vector<std::uint32_t> weapons;
    for (const auto& e : entries_) if (e && e->guidance && !e->weaponEnded) weapons.push_back(e->info.id);
    for (const auto id : weapons) if (Entry* e=entry(id)) {
        const auto spec=e->guidance->spec();
        const double age=simTime_-e->info.launchTimeS;
        const auto s=pool_->states()[e->slot];
        const auto before=stepStates_[e->slot];
        e->origin.burning=s.thrustN[0]>.1;
        const double dt=options_.dt*options_.frameSkip;
        const double liveEnd=std::clamp(s.simTime-before.simTime,0.0,dt);
        bool burst=false;
        double fraction=1;
        std::uint32_t burstTarget=e->origin.holding;
        bool contactInitiation=false;
        if (e->origin.tracking) {
            const double ageBefore=std::max(0.0,age-dt);
            const Entry* held=entry(e->origin.holding);
            if (held) {
                const auto fuze=ms::weapon::sweptPathFuze(e->engagementPath,held->engagementPath,
                        ageBefore,dt,spec,liveEnd,s.crashed || s.diverged);
                if (fuze.valid && std::isfinite(fuze.missM)) {
                    e->origin.missM=std::isfinite(e->origin.missM)?std::min(e->origin.missM,fuze.missM):fuze.missM;
                    burst=fuze.triggered; fraction=fuze.fraction;
                }
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
                    const auto fuze=ms::weapon::sweptPathFuze(e->engagementPath,path,ageBefore,dt,spec,end,
                        s.crashed || s.diverged || (trajectory.expires && trajectory.liveDurationS<=liveEnd));
                    if (fuze.valid && std::isfinite(fuze.missM)) {
                        e->origin.missM=fuze.missM; burst=fuze.triggered; fraction=fuze.fraction;
                    }
                }
        }
        if (!e->contactFuze.consumed() && e->contactNoseFromOriginBodyM && !e->origin.jettisoned &&
            captureEngagementMotion_ && ms::weapon::validMotionPath(e->engagementPath) &&
            e->engagementPath.size()>=2 && e->engagementStates.size()==e->engagementPath.size() &&
            e->engagementCg.size()==e->engagementPath.size()) {
            double contactEnd=std::min(liveEnd,e->engagementPath.back().offsetS);
            bool exclusive=s.crashed || s.diverged;
            if (exclusive) {
                // Do not interpolate through a native arrest's corrected pose
                // or a divergent endpoint. This channel admits only its known
                // moving prefix; the cutoff is discrete, not a solved terrain TOI.
                contactEnd=std::min(contactEnd,e->engagementPath[e->engagementPath.size()-2].offsetS);
                if (s.crashed) if (const auto* source=dynamic_cast<const sim::TerrainStrikeSourceV1*>(&pool_->vehicle(e->slot))) {
                    sim::VehicleState incoming;
                    if (source->terrainStrikeStateV1(incoming) && std::isfinite(incoming.simTime))
                        contactEnd=std::min(contactEnd,std::max(0.0,incoming.simTime-e->engagementModelTimeS));
                }
            }
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
                    if (targetPath.size()<2) continue;
                    const double boundary=targetPath[targetPath.size()-2].offsetS;
                    if (boundary<=end) {end=boundary;exclusiveEnd=true;}
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
                const auto contact=ms::weapon::contactFuze(relative,geometry,
                    {std::max(0.0,age-dt),dt,spec.armingS,spec.lifeS,end,exclusiveEnd});
                if (contact.valid && contact.contact && (!firstContact.contact || contact.offsetS<firstContact.offsetS ||
                    (contact.offsetS==firstContact.offsetS && candidate->info.id<physicalVictim))) {
                    firstContact=contact;physicalVictim=candidate->info.id;
                }
            }
            // Commit the earliest physical contact even before arming. An
            // earlier proximity initiation retains its existing semantics;
            // exact initiation-time ties also preserve proximity priority.
            if (e->contactFuze.commit(firstContact) && (!burst || firstContact.fraction<fraction)) {
                burst=true;fraction=firstContact.fraction;burstTarget=physicalVictim;contactInitiation=true;
            }
        }
        if (burst) {
            Detonation det;
            det.store=e->origin.store; det.weapon=e->info.id;
            det.stepFraction=fraction;
            const auto burstState=engagementStateAt(*e,fraction*dt);
            for(int k=0;k<3;++k) {
                det.positionEcef[k]=burstState.positionEcef[k];
                det.axisEcef[k]=burstState.rotationBodyToEcef[k*3];
            }
            ms::geom::velocityEcef(burstState,det.velocityEcef);
            if (contactInitiation) if (const auto* victim=entry(burstTarget)) {
                const auto at=engagementStateAt(*victim,fraction*dt);
                e->origin.missM=length(delta(point(burstState.positionEcef),point(at.positionEcef)));
            }
            const auto damage=detonate(det);
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
            const auto pose=terminal(point(det.positionEcef));
            endedState.latitudeRad=pose.latitudeRad; endedState.longitudeRad=pose.longitudeRad; endedState.altitudeMslM=pose.heightM;
            endWeapon(*e,ms::WeaponEnd::Detonated,endedState); ended.push_back(e->info.id);
        } else if (!s.crashed && (age>=spec.lifeS || s.diverged)) {
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
    const ms::StoreCard* card=nullptr;
    for (const auto& s : e->ms.stores()) if (s.store==store && ms::weapon::onRailObservationAvailable(s.status)) {card=e->ms.storeCard(s.index);break;}
    if (!card || !infrared(card->seeker)) return 0;
    return onRailSeekerObservation(*e,*card);
}

std::uint32_t World::onRailSeekerObservation(const Entry& launcher,const ms::StoreCard& card) const {
    const auto& own=pool_->states()[launcher.slot];
    double best=card.value("launch_fov_deg",25.0)*ms::geom::kPi/180.0;
    std::uint32_t found=0;
    for (const auto& candidate : entries_) if (candidate && candidate->info.role==ms::Role::Aircraft && candidate->info.id!=launcher.info.id) {
        const auto& target=pool_->states()[candidate->slot];
        if (target.crashed || target.diverged) continue;
        const V r=delta(point(target.positionEcef),point(own.positionEcef)), tv=velocity(target);
        const double range=length(r);
        if (range<1) continue;
        double body[3], raw[3]={r.x,r.y,r.z}; ms::geom::toBody(own,raw,body);
        const double off=std::acos(std::clamp(body[0]/range,-1.0,1.0));
        const double aspect=dot(r,tv)/(range*std::max(length(tv),1.0));
        const double reach=ms::weapon::infraredAcquisitionRangeM(card.value("seeker_acquire_m",13000.0),aspect,target.afterburner[0]>.5);
        if (off>=best || range>reach) continue;
        const auto path=radioPath(launcher.info.id,candidate->info.id,{});
        if (path && path->terrainMasked) continue;
        best=off; found=candidate->info.id;
    }
    return found;
}
} // namespace fsim::session
