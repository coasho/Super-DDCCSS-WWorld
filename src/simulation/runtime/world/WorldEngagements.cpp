// Native engagement lifecycle. JSBSim alone integrates weapon motion.
#include "simulation/runtime/world/World.h"
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
        const auto& seeker=spec.seekerModel;
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
                    illuminator->pattern().beamwidthRad(false),illuminator->pattern().beamwidthRad(true),seeker)) return false;
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
            if (range<seeker.minimumRangeM || (!heat && range>spec.acquireM)) continue;
            const V tv=velocity(target);
            double acquire=spec.acquireM;
            if (heat) {
                const double aspect=dot(r,tv)/(range*std::max(length(tv),seeker.aspectSpeedFloorMs));
                acquire=ms::weapon::infraredAcquisitionRangeM(spec.acquireM,aspect,target.afterburner[0]>seeker.afterburnerThreshold,seeker);
            } else {
                const auto cross=radarCrossSection(e->info.id,candidate->info.id,{});
                if (cross && cross->modelled) acquire=ms::weapon::radarAcquisitionRangeM(acquire,cross->sigmaM2,true,seeker);
                if (ms::weapon::radarNotched(dot(r,in.up)/range,dot(tv,r)/range,seeker)) continue;
            }
            if (range>acquire) continue;
            // Terrain masking and the curved radio horizon also apply to the seeker's own sensor.
            ms::RadioPathOptions path;
            if (semi && illuminator) path.frequencyHz=illuminator->spec().frequencyHz;
            const auto propagation=radioPath(e->info.id,candidate->info.id,path);
            if (propagation && propagation->terrainMasked) continue;
            in.visible.push_back({candidate->info.id,point(target.positionEcef),tv,false,seeker.aircraftSignal,acquire});
        }
        for (const auto* dispenser : dispensers()) for (const auto& decoy : dispenser->decoys()) {
            if (heat != (decoy.kind==ms::countermeasure::Kind::Flare)) continue;
            const V p{decoy.position.x,decoy.position.y,decoy.position.z}, v{decoy.velocity.x,decoy.velocity.y,decoy.velocity.z};
            if (!illuminatedCandidate(p)) continue;
            const V r=delta(p,in.position); const double range=length(r);
            if (range<seeker.minimumRangeM || range>spec.acquireM || (!heat && ms::weapon::radarNotched(dot(r,in.up)/range,dot(v,r)/range,seeker))) continue;
            ms::rf::PathOptions path; path.ground=ground_.get();
            if (semi && illuminator) path.frequencyHz=illuminator->spec().frequencyHz;
            const auto propagation=ms::rf::computePath({s.latitudeRad,s.longitudeRad,s.altitudeMslM},terminal(p),path);
            if (propagation.terrainMasked) continue;
            // basis: assumed. Relative signature bloom/fade for deterministic seeker discrimination.
            const double age=std::max(0.0,simTime_-decoy.launchTimeS);
            const double signal=ms::weapon::decoySignal(age,heat,seeker);
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
    for (const auto& e : entries_) if (e && (e->guidance || e->contactNoseFromOriginBodyM) && !e->weaponEnded) weapons.push_back(e->info.id);
    for (const auto id : weapons) if (Entry* e=entry(id)) {
        auto spec=e->fuzeAdapter ? e->fuzeAdapter->spec() : e->guidance ? e->guidance->spec() : ms::weapon::Spec{};
        if (!e->guidance) { spec.armingS=0; spec.lifeS=std::numeric_limits<double>::max(); }
        const double age=simTime_-e->info.launchTimeS;
        const auto s=pool_->states()[e->slot];
        e->origin.burning=s.thrustN[0]>.1;
        const double dt=options_.dt*options_.frameSkip;
        const double ageBefore=e->fuzeAgeBoundaryS;
        e->fuzeAgeBoundaryS=ageBefore+dt;
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
        const auto evaluateFuze=[&](const std::vector<ms::weapon::MotionSample>& targetPath,double end,bool exclusive) {
            const auto fuze=e->fuzeAdapter
                ? e->fuzeAdapter->evaluateProximity(sensorPath,targetPath,
                    spec.fuzeModelVersion==1 ? ageBefore : std::max(0.0,age-dt),dt,end,exclusive,e->origin.holding)
                : spec.fuzeModelVersion==1
                ? e->fuzeTurn.evaluate(sensorPath,targetPath,ageBefore,dt,spec,end,exclusive,e->origin.holding)
                : ms::weapon::sweptPathFuze(sensorPath,targetPath,std::max(0.0,age-dt),dt,spec,end,exclusive);
            if (fuze.valid && std::isfinite(fuze.missM)) {
                e->origin.missM=std::isfinite(e->origin.missM)?std::min(e->origin.missM,fuze.missM):fuze.missM;
                if (spec.fuzeModelVersion==1) {
                    if (fuze.triggered) {
                        if (e->fuzeAdapter) e->fuzeAdapter->scheduleDelay(ageBefore+fuze.fraction*dt,e->origin.holding);
                        else e->fuzeDelay.schedule(ageBefore+fuze.fraction*dt,e->origin.holding,spec);
                    }
                } else {burst=fuze.triggered;fraction=fuze.fraction;}
            }
        };
        if (e->origin.tracking) {
            const Entry* held=entry(e->origin.holding);
            if (held) {
                const auto targetCutoff=capturedCutoff(*held);
                const double end=std::min(liveEnd,targetCutoff.first);
                evaluateFuze(capturedPrefix(held->engagementPath,targetCutoff.first),end,
                    ownCutoff.second || (targetCutoff.second && targetCutoff.first<=liveEnd));
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
                    evaluateFuze(path,end,s.crashed || s.diverged || (trajectory.expires && trajectory.liveDurationS<=liveEnd));
                }
        } else if (e->fuzeAdapter) e->fuzeAdapter->resetTurnHistory();
        else e->fuzeTurn.reset();
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
            double contactEnd=liveEnd;
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
            const bool armedContact=committed && e->guidance && !e->origin.jettisoned;
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
            const auto damage=detonateAt(det,mechanical ? &*mechanical : nullptr);
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
        } else if (e->guidance && !s.crashed && (age>=spec.lifeS || s.diverged)) {
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
    std::string error; const auto configured=configuredWeaponSpec(card,error);
    if (!configured) return 0;
    const auto& seeker=configured->seekerModel;
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
        const double reach=ms::weapon::infraredAcquisitionRangeM(configured->acquireM,aspect,target.afterburner[0]>seeker.afterburnerThreshold,seeker);
        if (off>=best || range>reach) continue;
        const auto path=radioPath(launcher.info.id,candidate->info.id,{});
        if (path && path->terrainMasked) continue;
        best=off; found=candidate->info.id;
    }
    return found;
}
} // namespace fsim::session
