// Native engagement lifecycle. JSBSim alone integrates weapon motion.
#include "session/World.h"
#include "foundation/io/AssetResolver.h"
#include "simulation/mission_systems/rf/Path.h"
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
    for (auto* dispenser : dispensers()) if (!dispenser->decoys().empty()) {
        const auto& p=dispenser->decoys().front().position;
        dispenser->step(dt, {-p.x,-p.y,-p.z});
    }
    retiredDispensers_.erase(std::remove_if(retiredDispensers_.begin(),retiredDispensers_.end(),
        [](const auto& d){return d->decoys().empty();}),retiredDispensers_.end());
    for (auto& e : entries_) if (e && e->guidance && !e->weaponEnded) {
        const auto& s=pool_->states()[e->slot];
        const auto spec=e->guidance->spec();
        const bool heat=infrared(spec.seeker);
        ms::weapon::Input in;
        in.position=point(s.positionEcef); in.velocity=velocity(s); in.dtS=dt;
        in.ageS=simTime_-e->info.launchTimeS;
        in.launchCue=heat?e->origin.target:0;
        in.up={std::cos(s.latitudeRad)*std::cos(s.longitudeRad),std::cos(s.latitudeRad)*std::sin(s.longitudeRad),std::sin(s.latitudeRad)};
        const auto* d=designation(e->origin.target);
        const Entry* launcher=entry(e->info.launcher);
        if (d && d->valid && launcher && d->owner==e->info.launcher) {
            in.supported=true;
            in.estimate=ms::weapon::Observation{d->id,point(d->estimate.positionEcef),velocity(d->estimate),false};
            const auto r=launcher->ms.radarStatus(d->payload);
            in.illuminated=r.illuminating && r.designatedTrack==d->track;
        }
        // Seeker observations, not omniscient updates: range, aspect, gimbal and Doppler gates.
        // basis: assumed. IR aspect/afterburner factors, 28 m/s notch and decoy response are game estimates.
        for (const auto& candidate : entries_) if (candidate && candidate->info.role==ms::Role::Aircraft &&
                candidate->info.id!=e->info.launcher && candidate->info.id!=e->info.id) {
            const auto& target=pool_->states()[candidate->slot];
            if (target.crashed || target.diverged) continue;
            const V r=delta(point(target.positionEcef),in.position);
            const double range=length(r);
            if (range<1.0 || range>spec.acquireM) continue;
            const V tv=velocity(target);
            double acquire=spec.acquireM;
            if (heat) {
                const double aspect=.5*(1.0+dot(r,tv)/(range*std::max(length(tv),1.0)));
                acquire*=.45+.55*std::clamp(aspect,0.0,1.0);
                if (target.afterburner[0]>.5) acquire*=1.35;
            } else {
                const auto cross=radarCrossSection(e->info.id,candidate->info.id,{});
                if (cross && cross->modelled) acquire*=std::clamp(std::pow(std::max(cross->sigmaM2,.001)/5.0,.25),.4,1.0);
                if (dot(r,in.up)/range<-.03 && std::fabs(dot(tv,r)/range)<28.0) continue;
            }
            if (range>acquire) continue;
            // Terrain masking and the curved radio horizon also apply to the seeker's own sensor.
            ms::RadioPathOptions path;
            const auto propagation=radioPath(e->info.id,candidate->info.id,path);
            if (propagation && propagation->terrainMasked) continue;
            in.visible.push_back({candidate->info.id,point(target.positionEcef),tv,false});
        }
        for (const auto* dispenser : dispensers()) for (const auto& decoy : dispenser->decoys()) {
            if (heat != (decoy.kind==ms::countermeasure::Kind::Flare)) continue;
            const V p{decoy.position.x,decoy.position.y,decoy.position.z}, v{decoy.velocity.x,decoy.velocity.y,decoy.velocity.z};
            const V r=delta(p,in.position); const double range=length(r);
            if (range<1 || range>spec.acquireM || (!heat && dot(r,in.up)/range<-.03 && std::fabs(dot(v,r)/range)<28)) continue;
            ms::rf::PathOptions path; path.ground=ground_.get();
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
        e->guidanceForce[0]=output.acceleration.x; e->guidanceForce[1]=output.acceleration.y; e->guidanceForce[2]=output.acceleration.z;
        if (output.active && !heat && spec.seeker!="semi_active_radar" &&
                (old.phase==ms::WeaponPhase::Midcourse || old.phase==ms::WeaponPhase::Unguided)) recordEngagement("activation",*e);
        if (old.tracking && !output.tracking) recordEngagement("track_loss",*e,old.holding);
        if (!old.tracking && output.tracking) recordEngagement("acquisition",*e,output.holding);
        if (output.holding!=old.holding && output.holding>=ms::kDecoyIds && output.holding<kDesignations)
            recordEngagement("decoy_capture",*e,output.holding);
    }
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
        bool burst=false;
        double fraction=1;
        if (e->origin.tracking && ms::weapon::armed(age,spec)) {
            const Entry* held=entry(e->origin.holding);
            if (held) {
                const auto& target=pool_->states()[held->slot];
                const double miss=ms::weapon::sweptMissDistance(point(before.positionEcef),point(s.positionEcef),
                        point(stepStates_[held->slot].positionEcef),point(target.positionEcef),&fraction);
                e->origin.missM=std::isfinite(e->origin.missM)?std::min(e->origin.missM,miss):miss;
                burst=miss<=spec.fuzeM;
            } else for (const auto* dispenser : dispensers()) for (const auto& d : dispenser->decoys())
                if (d.id==e->origin.holding) {
                    const V p{d.position.x,d.position.y,d.position.z};
                    const double dt=options_.dt*options_.frameSkip;
                    const V prior{p.x-d.velocity.x*dt,p.y-d.velocity.y*dt,p.z-d.velocity.z*dt};
                    const double miss=ms::weapon::sweptMissDistance(point(before.positionEcef),point(s.positionEcef),prior,p,&fraction);
                    e->origin.missM=miss; burst=miss<=spec.fuzeM;
                }
        }
        if (burst) {
            Detonation det;
            det.store=e->origin.store; det.weapon=e->info.id;
            det.stepFraction=fraction;
            for(int k=0;k<3;++k) {
                det.positionEcef[k]=before.positionEcef[k]+fraction*(s.positionEcef[k]-before.positionEcef[k]);
                det.axisEcef[k]=s.rotationBodyToEcef[k*3];
            }
            ms::geom::velocityEcef(s,det.velocityEcef);
            const auto damage=detonate(det);
            recordEngagement("detonation",*e,e->origin.holding);
            combatEvents_.back().positionEcef[0]=det.positionEcef[0];
            combatEvents_.back().positionEcef[1]=det.positionEcef[1];
            combatEvents_.back().positionEcef[2]=det.positionEcef[2];
            std::set<std::uint32_t> victims;
            for (const auto& hit : damage) if (hit.kind!=static_cast<std::uint8_t>(ipc::EngagementKind::Detonation) && hit.slot<entries_.size()) {
                const auto* victim=entries_[hit.slot].get();
                if (victim && victims.insert(victim->info.id).second)
                    recordEngagement("damage",*e,victim->info.id,"warhead",victim->crewLost);
            }
            auto endedState=s;
            std::copy_n(det.positionEcef,3,endedState.positionEcef);
            const auto pose=terminal(point(det.positionEcef));
            endedState.latitudeRad=pose.latitudeRad; endedState.longitudeRad=pose.longitudeRad; endedState.altitudeMslM=pose.heightM;
            endWeapon(*e,ms::WeaponEnd::Detonated,endedState); ended.push_back(e->info.id);
        } else if (age>=spec.lifeS || s.diverged || (age>5 && !e->origin.burning && s.airspeedTrueMs<180)) {
            recordEngagement("expiry",*e,0,s.diverged?"flight_diverged":age>=spec.lifeS?"battery_expired":"energy_exhausted");
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
    for (const auto& s : e->ms.stores()) if (s.store==store && s.status==ms::StoreStatus::Loaded) {card=e->ms.storeCard(s.index);break;}
    if (!card || !infrared(card->seeker)) return 0;
    const auto& own=pool_->states()[e->slot];
    double best=card->value("launch_fov_deg",25.0)*ms::geom::kPi/180.0;
    std::uint32_t found=0;
    for (const auto& candidate : entries_) if (candidate && candidate->info.role==ms::Role::Aircraft && candidate->info.id!=vehicle) {
        const auto& target=pool_->states()[candidate->slot];
        if (target.crashed || target.diverged) continue;
        const V r=delta(point(target.positionEcef),point(own.positionEcef)), tv=velocity(target);
        const double range=length(r);
        if (range<1) continue;
        double body[3], raw[3]={r.x,r.y,r.z}; ms::geom::toBody(own,raw,body);
        const double off=std::acos(std::clamp(body[0]/range,-1.0,1.0));
        const double tail=.5*(1+dot(r,tv)/(range*std::max(length(tv),1.0)));
        const double reach=card->value("seeker_acquire_m",13000.0)*(.45+.55*std::clamp(tail,0.0,1.0))*(target.afterburner[0]>.5?1.35:1.0);
        if (off>=best || range>reach) continue;
        const auto path=radioPath(vehicle,candidate->info.id,{});
        if (path && path->terrainMasked) continue;
        best=off; found=candidate->info.id;
    }
    return found;
}
} // namespace fsim::session
