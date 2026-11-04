// MS-7g's damage part in the world (docs/mission-systems.md, G3 - G7): a warhead going off and what it breaks on the
// aircraft near it, then, step by step, what that does as they fly on - their holed tanks leaking (and burning),
// what the damage takes off the aerodynamic force (World::preStep), each section's root against its strength left,
// a failed one leaving with its mass. The physics is ms/damage's; the chances are each vehicle's own damage stream.

#include "simulation/runtime/world/World.h"
#include "simulation/runtime/world/WorldLiquidMaterials.h"

#include "simulation/mission_systems/damage/Debris.h"
#include "simulation/mission_systems/damage/Impact.h"

#include "foundation/core/Log.h"
#include "foundation/core/Units.h"
#include "foundation/io/AssetResolver.h"

#include <algorithm>
#include <cstring>
#include <set>

namespace fsim::session {

struct WorldDamageContinuationV1 {
    struct Pending {
        Detonation source;
        ms::WeaponRecord correlation;
        // Historical source reference for the existing Hit.target wire field.
        // The immutable correlation ID remains authoritative after slot reuse.
        std::optional<std::uint32_t> sourceStreamSlot;
        std::uint64_t sourceGeneration=0;
        std::string diagnosticName;
        double diagnosticExplosiveKg=0;
        std::uint32_t diagnosticTopology=0;
        ms::damage::FragmentFieldV1 field;
        ms::damage::SceneEnvironmentV1 environment;
        double sampleTimeS=0,creationBoundaryS=0,lifetimeS=1;
        double lastPublicationS=std::numeric_limits<double>::quiet_NaN();
    };
    std::vector<Pending> pending;
};

namespace {

constexpr std::uint64_t kDamageStream = 0x444D'4147ull; // "DMAG": a stream of its own, so no other draw moves
constexpr std::uint64_t kFragmentStream = 0x4652'4147ull; // "FRAG": one source population per physical release.
constexpr double kLbsToKg = 0.45359237, kLbfToN = 4.4482216152605, kPsfToPa = 47.880258888889;
constexpr double kSlugFt3ToKgM3 = 515.378818;
constexpr double kG = 9.80665;
constexpr double kRateReported = 0.1; ///< a fire's rate recorded again when it has moved by a tenth
constexpr double kSeparationMs = 5.0; ///< a failed section's push away from its root (an estimate: the break's spring)
/// A section a hit breaks off leaves with its own share of the hit's push, no faster than this from its aircraft (an
/// estimate: a light panel's share of a near burst would otherwise be hundreds of metres a second - it tears, it is
/// not thrown whole)
constexpr double kThrownMostMs = 60.0;
/// A holed tank's fuel is thrown out the way the fragments went through it (hydrodynamic ram: the spray leaves the far
/// wall) at this share of their speed (an estimate: some tens of metres a second behind fragments at 1.5 - 2 km/s)
constexpr double kRamSprayShare = 0.03;

ms::damage::V3 v3(const double* a) { return {a[0], a[1], a[2]}; }
ms::damage::V3 rotate(const sim::VehicleState& s, ms::damage::V3 v) {
    const auto* r=s.rotationBodyToEcef;
    return {r[0]*v.x+r[1]*v.y+r[2]*v.z,r[3]*v.x+r[4]*v.y+r[5]*v.z,r[6]*v.x+r[7]*v.y+r[8]*v.z};
}
ms::damage::V3 unrotate(const sim::VehicleState& s, ms::damage::V3 v) {
    const auto* r=s.rotationBodyToEcef;
    return {r[0]*v.x+r[3]*v.y+r[6]*v.z,r[1]*v.x+r[4]*v.y+r[7]*v.z,r[2]*v.x+r[5]*v.y+r[8]*v.z};
}


// a section's place in its model's structural frame (m: x aft, y right, z up) from body axes off its origin
void structural(const ms::damage::Airframe& a, const ms::damage::V3& body, double out[3]) {
    out[0] = a.originDesign.x - body.x;
    out[1] = a.originDesign.y + body.y;
    out[2] = a.originDesign.z - body.z;
}

void synchronizeDetachedFuel(sim::FlightModel& model,const ms::damage::VehicleDamage& damage) {
    const auto& frame=damage.airframe();double total=0;ms::damage::V3 weighted;
    std::set<int> used;
    for(const auto& section:frame.sections)for(const auto& tank:section.tanks)
        if(const auto contents=damage.detachedFuelKg(tank.index);contents&&used.insert(tank.index).second) {
            total+=*contents;weighted+=tank.centre*(*contents);
        }
    model.beginMassChange();model.property("inertia/pointmass-weight-lbs").set(total/kLbsToKg);
    if(total>0) {
        double location[3];structural(frame,weighted*(1/total),location);
        constexpr double inchesPerM=39.37007874015748;
        const char* axes[]={"X","Y","Z"};
        for(std::size_t axis=0;axis<3;++axis)
            model.property(std::string("inertia/pointmass-location-")+axes[axis]+"-inches").set(location[axis]*inchesPerM);
    }
    model.endMassChange(nullptr,nullptr);
}

} // namespace

std::optional<World::Entry::Matter> World::captureMatter(const Entry& e, sim::FlightModel& model) {
    const auto* source=dynamic_cast<const sim::RigidMassSourceV1*>(&model);
    Entry::Matter out;
    if (!source || !source->rigidMassPropertiesV1(out.rigid)) return std::nullopt;
    static constexpr const char* paths[]={"atmosphere/P-psf","atmosphere/T-R","atmosphere/rho-slugs_ft3"};
    auto& reads=e.matterAtmosphere;
    for (std::size_t k=0;k<reads.size();++k)
        if (!reads[k].valid()) reads[k]=model.property(paths[k]);
    if (!reads[0].valid() || !reads[1].valid() || !reads[2].valid()) return std::nullopt;
    out.pressurePa=reads[0].get()*kPsfToPa; out.temperatureK=reads[1].get()/1.8; out.densityKgM3=reads[2].get()*kSlugFt3ToKgM3;
    if (!(out.pressurePa>0) || !(out.temperatureK>0) || !(out.densityKgM3>=0) ||
        !std::isfinite(out.pressurePa) || !std::isfinite(out.temperatureK) || !std::isfinite(out.densityKgM3)) return std::nullopt;
    if(!e.matterFuelBound) {
        std::map<std::size_t,sim::PropertyHandle> tanks;
        for(const auto& property:model.properties("propulsion")) {
            const auto& name=property.first;std::optional<std::size_t> index;
            if(name=="tank/contents-lbs")index=0;
            else if(name.compare(0,5,"tank[")==0) {
                const auto end=name.find(']',5);
                if(end!=std::string::npos&&end>5&&name.substr(end)=="]/contents-lbs") {
                    std::size_t value=0;bool valid=true;
                    for(std::size_t k=5;k<end;++k) {
                        if(name[k]<'0'||name[k]>'9'){valid=false;break;}
                        value=value*10+static_cast<std::size_t>(name[k]-'0');
                        if(value>65535){valid=false;break;}
                    }
                    if(valid)index=value;
                }
            }
            if(index)tanks.emplace(*index,model.property("propulsion/tank["+std::to_string(*index)+"]/contents-lbs"));
        }
        e.matterFuelReads.assign(tanks.begin(),tanks.end());e.matterFuelBound=true;
    }
    for(const auto& item:e.matterFuelReads) {
        if(out.tankFuelKg.size()<=item.first)out.tankFuelKg.resize(item.first+1);
        if(!item.second.valid())continue;
        const double contents=item.second.get()*kLbsToKg;
        if(std::isfinite(contents)&&contents>=0)out.tankFuelKg[item.first]=contents;
    }
    if(e.damage&&e.damageOwnedSection)for(const auto& tank:e.damage->airframe().sections[static_cast<std::size_t>(*e.damageOwnedSection)].tanks) {
        if(tank.index<0||tank.index>65535)continue;
        const auto index=static_cast<std::size_t>(tank.index);
        if(out.tankFuelKg.size()<=index)out.tankFuelKg.resize(index+1);
        // Detached fuel belongs to the real child damage state and native
        // point mass, rather than a tank absent from its dry piece XML.
        if(const auto contents=e.damage->detachedFuelKg(tank.index))out.tankFuelKg[index]=*contents;
    }
    else if(e.damageOwnedSection&&e.liquidMaterials) {
        // The generated native piece's first point mass is its fuel. Physical
        // reset restores that actual point mass to zero before lazy damage
        // state construction. A measured zero aggregate uniquely establishes
        // zero for each nonnegative owned tank; a positive unknown aggregate
        // cannot be split provisionally among native tank identities.
        std::optional<double> fuel;
        for(const auto& property:model.properties("inertia"))
            if(property.first=="pointmass-weight-lbs"||property.first=="pointmass-weight-lbs[0]")fuel=property.second;
        if(fuel&&*fuel==0)for(const auto& tank:e.liquidMaterials->tanks)
            if(tank.section==*e.damageOwnedSection) {
                const auto index=static_cast<std::size_t>(tank.index);
                if(out.tankFuelKg.size()<=index)out.tankFuelKg.resize(index+1);
                out.tankFuelKg[index]=0;
            }
    }
    return out;
}

std::optional<World::Entry::Matter> World::engagementMatterAt(const Entry& e, double offsetS) const {
    const auto& path=e.engagementPath; const auto& values=e.engagementMatter;
    if (!std::isfinite(offsetS) || offsetS<0 || values.size()!=path.size() ||
        e.engagementStates.size()!=path.size() || !ms::weapon::validMotionPath(path)) return std::nullopt;
    for (std::size_t i=0;i<path.size();++i) {
        if (i && offsetS>path[i-1].offsetS && !e.engagementStates.front().crashed &&
            (e.engagementStates[i].crashed || e.engagementStates[i].diverged)) return std::nullopt;
        if (offsetS==path[i].offsetS) return values[i];
        if (offsetS<path[i].offsetS) {
            if (!i || !values[i-1] || !values[i]) return std::nullopt;
            const double span=path[i].offsetS-path[i-1].offsetS;
            if (!(span>0)) return std::nullopt;
            const double f=(offsetS-path[i-1].offsetS)/span;
            const auto& a=*values[i-1]; const auto& b=*values[i]; auto out=a;
            const auto mix=[&](double x,double y){return std::fma(f,y-x,x);};
            out.rigid.massKg=mix(a.rigid.massKg,b.rigid.massKg);
            for (int k=0;k<9;++k) out.rigid.inertiaBodyKgM2[k]=mix(a.rigid.inertiaBodyKgM2[k],b.rigid.inertiaBodyKgM2[k]);
            out.pressurePa=mix(a.pressurePa,b.pressurePa); out.temperatureK=mix(a.temperatureK,b.temperatureK);
            out.densityKgM3=mix(a.densityKgM3,b.densityKgM3);
            out.tankFuelKg.assign(std::max(a.tankFuelKg.size(),b.tankFuelKg.size()),std::nullopt);
            for(std::size_t tank=0;tank<std::min(a.tankFuelKg.size(),b.tankFuelKg.size());++tank)
                if(a.tankFuelKg[tank]&&b.tankFuelKg[tank])out.tankFuelKg[tank]=mix(*a.tankFuelKg[tank],*b.tankFuelKg[tank]);
            return out;
        }
    }
    if (e.engagementStates.front().crashed && e.engagementStates.back().crashed) return values.back();
    return std::nullopt;
}

std::optional<World::ContactTransfer> World::contactTransfer(const Entry& weapon, std::uint32_t victimId,
    const ms::weapon::ContactFuzeResult& contact) const {
    const auto* victim=entry(victimId);
    if (!victim || !contact.valid || !contact.contact) return std::nullopt;
    const auto victimCg=engagementCgAt(*victim,contact.offsetS);
    if (!victimCg) return std::nullopt;
    const auto b=engagementStateAt(*victim,contact.offsetS);
    ContactTransfer out; out.victim=victimId; out.fraction=contact.fraction; out.pointBody=contact.pointBody;
    out.pointEcef=v3(b.positionEcef)+rotate(b,contact.pointBody-*victimCg);
    const auto frame=airframes_.find(victim->info.aircraft);
    if (frame==airframes_.end()) return std::nullopt;
    if (contact.kind==ms::weapon::ContactKind::Panel) {
        if (contact.index>=frame->second->panels.size()) return std::nullopt;
        out.section=frame->second->panels[contact.index].section;
    } else {
        if (contact.index>=frame->second->components.size()) return std::nullopt;
        out.section=frame->second->components[contact.index].section;
    }
    const auto body=[&](const sim::VehicleState& s,const Entry::Matter& matter) {
        ms::damage::ContactBody result; result.massKg=matter.rigid.massKg;
        result.velocity=rotate(s,v3(s.velocityBodyMs)); result.angularVelocity=rotate(s,v3(s.angularRateBodyRadS));
        result.arm=out.pointEcef-v3(s.positionEcef);
        const auto* r=s.rotationBodyToEcef; const auto* inertia=matter.rigid.inertiaBodyKgM2;
        for (int row=0;row<3;++row) for (int col=row;col<3;++col) {
            double value=0;
            for (int i=0;i<3;++i) for (int j=0;j<3;++j) value+=r[3*row+i]*inertia[3*i+j]*r[3*col+j];
            result.inertia[3*row+col]=result.inertia[3*col+row]=value;
        }
        return result;
    };
    // Detection uses captured geometry. Response is a discrete end-of-decision
    // solve using the actual application mass, inertia, velocities and common
    // world point. Moving an old impulse to new arms can create kinetic energy.
    const auto weaponMatter=captureMatter(weapon,pool_->vehicle(weapon.slot));
    const auto victimMatter=captureMatter(*victim,pool_->vehicle(victim->slot));
    if (!weaponMatter || !victimMatter) return std::nullopt;
    sim::VehicleState applicationA,applicationB;
    pool_->vehicle(weapon.slot).state(applicationA); pool_->vehicle(victim->slot).state(applicationB);
    const auto impulse=ms::damage::inelasticBodyContact(body(applicationA,*weaponMatter),body(applicationB,*victimMatter),rotate(b,contact.normalBody));
    if (!impulse.valid || !impulse.approaching) return std::nullopt;
    out.impulseEcef=impulse.impulseB; out.weaponImpulseEcef=impulse.impulseA; out.weaponAngularEcef=impulse.angularImpulseA;
    out.dissipatedJ=impulse.dissipatedJ; out.closingMs=impulse.closingMs;
    return out;
}

void World::contactReaction(Entry& weapon,const ContactTransfer& contact) {
    sim::VehicleState now; pool_->vehicle(weapon.slot).state(now);
    const auto j=unrotate(now,contact.weaponImpulseEcef);
    const auto l=unrotate(now,ms::damage::cross(contact.pointEcef-v3(now.positionEcef),contact.weaponImpulseEcef));
    const double linear[]={j.x,j.y,j.z}, angular[]={l.x,l.y,l.z};
    pool_->vehicle(weapon.slot).applyImpulse(linear,angular);
    pool_->refreshState(weapon.slot);
    LOG_INFO("session") << "body contact weapon=" << weapon.info.id << " victim=" << contact.victim
        << " closing_m_s=" << contact.closingMs << " dissipated_j=" << contact.dissipatedJ
        << " impulse_n_s=" << ms::damage::length(contact.impulseEcef);
}

void World::contactDamage(Entry& weapon,const ContactTransfer& contact) {
    auto* victim=entry(contact.victim);
    const double offset=contact.fraction*options_.dt*options_.frameSkip;
    if (!victim) return;
    const auto cg=engagementCgAt(*victim,offset);
    const auto matter=engagementMatterAt(*victim,offset);
    if (!cg || !matter || !ensureDamage(*victim)) return;
    const auto s=engagementStateAt(*victim,offset);
    const auto j=unrotate(s,contact.impulseEcef);
    ms::damage::Impact impact;
    impact.impulseBody=j; impact.angularImpulseBody=ms::damage::cross(contact.pointBody,j);
    const double centre[]={cg->x,cg->y,cg->z};
    Detonation cause; cause.weapon=weapon.info.id; cause.store=weapon.origin.store;
    const auto events=applyDamage(*victim,cause,impact,s,centre,matter->pressurePa,
        {{contact.section,contact.pointBody,j}});
    for (const auto& event : events) engage(event);
    recordEngagement("damage",weapon,victim->info.id,"body_contact",victim->crewLost);
}

ipc::EngagementEvent World::engagementEvent(const Entry& e, ipc::EngagementKind kind, const std::string& name) const {
    ipc::EngagementEvent ev;
    ev.kind = static_cast<std::uint8_t>(kind);
    ev.slot = static_cast<std::uint32_t>(e.slot);
    ev.simTime = simTime_;
    const sim::VehicleState& s = pool_->states()[e.slot];
    for (int k = 0; k < 3; ++k) ev.positionEcef[k] = s.positionEcef[k];
    // (its velocity in ECEF: the body's, turned by its attitude - the rotation's columns are the body axes)
    const double* R = s.rotationBodyToEcef;
    for (int r = 0; r < 3; ++r) ev.velocityEcef[r] = R[3 * r] * s.velocityBodyMs[0] + R[3 * r + 1] * s.velocityBodyMs[1] + R[3 * r + 2] * s.velocityBodyMs[2];
    std::snprintf(ev.name, sizeof ev.name, "%s", name.c_str());
    return ev;
}

void World::damageFlight(Entry& e, ms::damage::Flight& f) {
    auto& r = e.damageReads;
    const double weightN = r.weight.get() * kLbfToN;
    for (int k = 0; k < 3; ++k) {
        (&f.aeroForceN.x)[k] = r.aero[static_cast<std::size_t>(k)].get() * kLbfToN;
        (&f.specificForceMs2.x)[k] = weightN > 0.0 ? r.total[static_cast<std::size_t>(k)].get() * kLbfToN / (weightN / kG) : 0.0;
        (&f.rateDotRadS2.x)[k] = e.angularAcceleration[static_cast<std::size_t>(k)].get();
    }
    const sim::VehicleState& s = e.working;
    f.airVelocityBodyMs={s.airspeedTrueMs*std::cos(s.alphaRad)*std::cos(s.betaRad),
        s.airspeedTrueMs*std::sin(s.betaRad),s.airspeedTrueMs*std::sin(s.alphaRad)*std::cos(s.betaRad)};
    f.rateRadS = v3(s.angularRateBodyRadS);
    f.dynamicPressurePa = r.qbar.get() * kPsfToPa;
    f.tankKg.resize(r.tanks.size());
    for (std::size_t t = 0; t < r.tanks.size(); ++t) f.tankKg[t] = r.tanks[t].get() * kLbsToKg;
    if(e.info.role==ms::Role::Debris&&e.damage)
        for(const auto& section:e.damage->airframe().sections)for(const auto& tank:section.tanks)
            if(const auto contents=e.damage->detachedFuelKg(tank.index)) {
                const auto index=static_cast<std::size_t>(tank.index);
                if(f.tankKg.size()<=index)f.tankKg.resize(index+1);
                f.tankKg[index]=*contents;
            }
}

std::vector<ipc::EngagementEvent> World::applyDamage(Entry& e, const Detonation& d,
    const ms::damage::Impact& impact, const sim::VehicleState& s, const double* cg,
    double pressurePa, const std::vector<ms::damage::VehicleDamage::PointImpulse>& contact,
    bool materialOriginMoment,bool liquidAware) {
    const Entry* weapon=entry(d.weapon);
    return applyDamageWithSource(e,d,impact,s,cg,pressurePa,contact,
        weapon?std::optional<std::uint32_t>{static_cast<std::uint32_t>(weapon->slot)}:std::nullopt,
        weapon?weapon->info.generation:0,materialOriginMoment,liquidAware);
}

std::vector<ipc::EngagementEvent> World::applyDamageWithSource(Entry& e, const Detonation& d,
    const ms::damage::Impact& impact, const sim::VehicleState& s, const double* cg,
    double pressurePa, const std::vector<ms::damage::VehicleDamage::PointImpulse>& contact,
    std::optional<std::uint32_t> sourceStreamSlot,std::uint64_t sourceGeneration,
    bool materialOriginMoment,bool liquidAware) {
    std::vector<ipc::EngagementEvent> out;
    auto& model=pool_->vehicle(e.slot);
    const auto af=airframes_.find(e.info.aircraft);
    if(af==airframes_.end()||!af->second||!e.damage) {
        if(!e.liquidMaterials||e.liquidMaterials->tanks.empty()||
           std::any_of(e.liquidMaterials->tanks.begin(),e.liquidMaterials->tanks.end(),[](const auto& tank){return tank.section!=-1;}))return out;
        // Liquid-only work has no authored skin/components/failure/fire model.
        // Apply the same scene-origin impulse convention to the real native body.
        sim::VehicleState now;model.state(now);
        const auto inCurrentBody=[&](ms::damage::V3 value) {
            const double body[3]={value.x,value.y,value.z};double ecef[3],current[3];
            ms::geom::toEcef(s,body,ecef);ms::geom::toBody(now,ecef,current);return v3(current);
        };
        const auto impulse=inCurrentBody(impact.impulseBody);
        const auto angular=inCurrentBody(impact.angularImpulseBody-ms::damage::cross(v3(cg),impact.impulseBody));
        const double linear[3]={impulse.x,impulse.y,impulse.z},torque[3]={angular.x,angular.y,angular.z};
        model.applyImpulse(linear,torque);
        return out;
    }
    const auto* R=s.rotationBodyToEcef;
    // where the burst was from its origin (body axes), and what struck each section's own panels: the fragments'
    // momentum, and the blast's impulse on each face, away from the burst
    const ms::damage::V3 burstBody = impact.fromBody * impact.missM;
    std::vector<ms::damage::V3> sectionImpulse(af->second->sections.size());
    for (const auto& ph : impact.panels) {
        if (ph.panel < 0 || static_cast<std::size_t>(ph.panel) >= af->second->panels.size()) continue;
        const auto& panel = af->second->panels[static_cast<std::size_t>(ph.panel)];
        if (panel.section < 0 || static_cast<std::size_t>(panel.section) >= sectionImpulse.size()) continue;
        sectionImpulse[static_cast<std::size_t>(panel.section)] +=
            ph.momentumNs + ms::damage::panelBlastImpulse(panel,burstBody,ph.impulsePas);
    }
    for (const auto& load : contact)
        if (load.section>=0 && static_cast<std::size_t>(load.section)<sectionImpulse.size())
            sectionImpulse[static_cast<std::size_t>(load.section)] += load.momentumNs;
    captureWeaponImpactDiagnosticsV1(d.weapon,sourceGeneration,e,impact);
    const double hitKg = model.energy().massKg;
    // Assessment vectors are in the encounter-time body frame. Apply their
    // inertial direction in the model's current body frame after the batch.
    sim::VehicleState currentState; model.state(currentState);
    double applicationCg[3];
    const bool hasApplicationCg=model.cgFromOriginBodyM(applicationCg);
    const auto currentBody=[&](ms::damage::V3 v) {
        const double body[3]={v.x,v.y,v.z}; double ecef[3],now[3];
        ms::geom::toEcef(s,body,ecef); ms::geom::toBody(currentState,ecef,now);
        return v3(now);
    };
    const auto currentImpulse=currentBody(impact.impulseBody);
    {
        // the blast and the fragments push and turn it (about its CG: the impulse's moment about the origin less
        // the CG's offset's)
        const ms::damage::V3 J = currentImpulse;
        auto worldTorque=rotate(s,impact.angularImpulseBody-ms::damage::cross(v3(cg),impact.impulseBody));
        const auto displacement=v3(s.positionEcef)-v3(currentState.positionEcef);
        if(materialOriginMoment) {
            // Scene moments already follow each moving material origin. A
            // fixed-world translation here would turn delayed orbital motion
            // into spin. Explicit mechanical contact retains its own previous
            // fixed-point transport even when combined with a scene burst.
            for(const auto& load:contact)
                worldTorque+=ms::damage::cross(displacement,rotate(s,load.momentumNs));
        } else {
            // Existing assess providers and direct collision callers retain
            // their historical fixed-world encounter-point convention.
            worldTorque+=ms::damage::cross(displacement,rotate(s,impact.impulseBody));
        }
        const ms::damage::V3 L = unrotate(currentState,worldTorque);
        const double linear[3] = {J.x, J.y, J.z}, angular[3] = {L.x, L.y, L.z};
        model.applyImpulse(linear, angular);
    }
    if (impact.fragmentsStriking > 0) {
        auto hit = engagementEvent(e, ipc::EngagementKind::Hit, d.store);
        hit.target = sourceStreamSlot ? *sourceStreamSlot + 1u : 0u;
        hit.index = static_cast<std::uint32_t>(impact.fragmentsStriking);
        out.push_back(hit);
    }
    ms::damage::Flight f;
    damageFlight(e, f);
    const auto& a = *af->second;
    sim::VehicleState afterImpulse; model.state(afterImpulse);
    auto changes=e.damage->apply(impact,pressurePa,e.damageRng,contact);
    if(liquidAware) {
        auto later=e.damage->applyLiquidEnergyV1(impact,pressurePa,e.damageRng);
        changes.insert(changes.end(),later.begin(),later.end());
    }
    for (const auto& c : changes) {
        using K = ms::damage::Change::Kind;
        const std::string component = c.component >= 0 ? a.components[static_cast<std::size_t>(c.component)].name : std::string();
        switch (c.kind) {
        case K::Fire: {
            auto ev = engagementEvent(e, ipc::EngagementKind::Fire, component);
            if (c.component >= 0) { // (where its component is on it)
                const auto& centre = a.components[static_cast<std::size_t>(c.component)].centre;
                // Component coordinates are relative to the fixed model
                // origin, not the CG. Keep pose, CG and rigid-point velocity
                // at the same captured encounter epoch as the assessment.
                const auto point=ms::weapon::materialPoint(s,v3(cg),centre);
                if(!point.valid) {LOG_WARN("session") << "fire component point unavailable for " << e.info.name;break;}
                for(int k=0;k<3;++k) {
                    ev.positionEcef[k]=(&point.position.x)[k];
                    ev.velocityEcef[k]=(&point.velocity.x)[k];
                }
            }
            ev.amountKg = c.tank>=0&&static_cast<std::size_t>(c.tank)<f.tankKg.size()?
                std::min(c.spillKg,f.tankKg[static_cast<std::size_t>(c.tank)]):c.spillKg;
            if (c.spillKg > 0.0 && c.component >= 0) {
                // what it throws out at once goes the way the fragments went through it - from the burst, through
                // the tank, out of its far side - on top of the aircraft's own motion: the fireball is laid along
                // the aircraft's path, and off it to the side away from the burst
                double fastest = 0.0;
                for (const auto& h : impact.components)
                    if (h.component == c.component) fastest = std::max(fastest, h.fastestMs);
                const ms::damage::V3 through = ms::damage::normalized(a.components[static_cast<std::size_t>(c.component)].centre - burstBody) *
                                               (kRamSprayShare * fastest);
                for (int r = 0; r < 3; ++r) ev.velocityEcef[r] += R[3 * r] * through.x + R[3 * r + 1] * through.y + R[3 * r + 2] * through.z;
            }
            if (c.tank >= 0 && static_cast<std::size_t>(c.tank) < f.tankKg.size()) {
                // what it throws out at once leaves its tank; what leaks on burns on
                const double kg = std::max(f.tankKg[static_cast<std::size_t>(c.tank)] - c.spillKg, 0.0);
                if(e.damage->detachedFuelKg(c.tank)) {
                    e.damage->removeDetachedFuelKg(c.tank,c.spillKg);
                    synchronizeDetachedFuel(model,*e.damage);
                } else e.damageReads.tanks[static_cast<std::size_t>(c.tank)].set(kg/kLbsToKg);
                ev.massKg = e.damage->leakKgS(c.tank, kg, s.loadFactor, f.dynamicPressurePa);
                ev.rateKgS = std::min(ev.massKg, ms::damage::kBurnMostKgS); // (what burns of it; the rest its mist)
                e.fireRates[c.tank] = ev.massKg;
            } else if (c.engine >= 0) {
                // an engine's fire: the fuel its feed was flowing as it failed (an estimate: until it is shut off)
                ev.rateKgS = model.property("propulsion/engine[" + std::to_string(c.engine) + "]/fuel-flow-rate-pps").get() * kLbsToKg;
                ev.massKg = ev.rateKgS;
            }
            out.push_back(ev);
            break;
        }
        case K::EngineOut: {
            model.seizeEngine(c.engine);
            auto ev = engagementEvent(e, ipc::EngagementKind::EngineOut, component);
            ev.index = static_cast<std::uint32_t>(std::max(c.engine, 0));
            out.push_back(ev);
            break;
        }
        case K::ControlLost: {
            const int n = model.failChannel(c.channel);
            if (n == 0) LOG_WARN("session") << e.info.name << ": no actuator of its '" << c.channel << "' channel to fail";
            out.push_back(engagementEvent(e, ipc::EngagementKind::ControlLost, c.channel));
            break;
        }
        case K::CrewLost:
            e.crewLost = true;
            out.push_back(engagementEvent(e, ipc::EngagementKind::CrewLost, component));
            break;
        case K::Separated: { // (the blast's impulse broke its root at once)
            Thrown thrown;
            if (hasApplicationCg) thrown.sharedCgFromOriginBodyM=v3(applicationCg);
            const auto J = currentBody(sectionImpulse[static_cast<std::size_t>(c.section)]);
            thrown.impulseBody[0] = J.x, thrown.impulseBody[1] = J.y, thrown.impulseBody[2] = J.z;
            if (std::isfinite(hitKg) && hitKg > 0.0)
                for (int k = 0; k < 3; ++k) thrown.sharedBody[k] = (&currentImpulse.x)[k] / hitKg;
            for (int k=0;k<3;++k) thrown.sharedAngularBody[k]=afterImpulse.angularRateBodyRadS[k]-currentState.angularRateBodyRadS[k];
            separatedAt(e, c.section, nullptr, &thrown, &out);
            break;
        }
        case K::PanelTorn: {
            auto ev = engagementEvent(e, ipc::EngagementKind::PanelTorn, a.sections[static_cast<std::size_t>(c.section)].name);
            ev.index = static_cast<std::uint32_t>(c.panel);
            out.push_back(ev);
            break;
        }
        default:
            break;
        }
    }
    pool_->refreshState(e.slot);
    return out;
}

std::vector<ipc::EngagementEvent> World::detonate(const Detonation& d) { return detonateAt(d,nullptr); }

std::vector<ipc::EngagementEvent> World::detonateCapturedV1(const Detonation& d) {
    const auto* source=entry(d.weapon);
    if(!source||source->info.role!=ms::Role::Weapon||!std::isfinite(d.stepFraction)||d.stepFraction<0||d.stepFraction>1||
       !ms::weapon::validMotionPath(source->engagementPath)||
       (source->warheadAdapter&&!dynamic_cast<const ms::damage::WarheadSceneExecutionV2*>(source->warheadAdapter.get()))) {
        LOG_WARN("session") << "captured detonation requires a live scene-capable weapon and captured native fraction";return {};
    }
    return detonateAt(d,nullptr,true);
}

std::vector<ipc::EngagementEvent> World::detonateAt(const Detonation& d, const ContactTransfer* contact,bool causalRequested) {
    std::vector<ipc::EngagementEvent> out;
    if (storeCards_.empty()) {
        std::vector<std::string> errors;
        if (const auto library = io::AssetResolver{}.find("stores")) {
            storeCards_ = ms::readStoreLibrary(*library, errors);
            for (const auto& [id,card]:storeCards_) {
                std::string error;
                if (!bindPayloadProfile(card,*library,error)) errors.push_back(id+": "+error);
            }
        }
        for (const auto& error : errors) LOG_WARN("session") << "stores: " << error;
    }
    const auto card = storeCards_.find(d.store);
    const Entry* weapon = entry(d.weapon);
    std::unique_ptr<ms::damage::WarheadAdapter> scriptedAdapter;
    const ms::damage::WarheadAdapter* adapter=weapon && weapon->origin.store==d.store ? weapon->warheadAdapter.get() : nullptr;
    if (!adapter && card!=storeCards_.end()) {
        std::string error;
        const auto profile=payloadProfile(*card->second,error);
        if (!profile) {LOG_WARN("session") << "detonate component profile: " << error;return out;}
        if (profile->explicitProfile()) {
            ms::components::ProviderContextV1 context{*card->second,*profile,{},std::nullopt};
            scriptedAdapter=payloadProviders_.warhead.create(*profile->find("warhead"),context,error);
            if (!scriptedAdapter) {LOG_WARN("session") << "detonate component selection: " << error;return out;}
            adapter=scriptedAdapter.get();
        }
    }
    const auto warhead = adapter ? std::optional<ms::damage::Warhead>{adapter->model()}
        : card == storeCards_.end() ? std::nullopt : ms::damage::warheadOf(*card->second);
    if (!warhead) {
        LOG_WARN("session") << "detonate: '" << d.store << "' has no warhead on its card";
        return out;
    }
    ipc::EngagementEvent burst;
    burst.kind = static_cast<std::uint8_t>(ipc::EngagementKind::Detonation);
    burst.slot = weapon ? static_cast<std::uint32_t>(weapon->slot) : 0u;
    burst.simTime = simTime_;
    for (int k = 0; k < 3; ++k) burst.positionEcef[k] = d.positionEcef[k], burst.velocityEcef[k] = d.velocityEcef[k];
    burst.amountKg = warhead->tntKg, burst.massKg = warhead->totalKg;
    std::snprintf(burst.name, sizeof burst.name, "%s", d.store.c_str());
    const std::size_t burstAt = out.size();
    out.push_back(burst);
    double nearest = std::numeric_limits<double>::infinity();

    ms::damage::Burst b;
    b.at = v3(d.positionEcef), b.velocity = v3(d.velocityEcef), b.axis = v3(d.axisEcef);
    if (weapon && std::isfinite(d.stepFraction))
        if (const auto point=weaponPointAt(*weapon,std::clamp(d.stepFraction,0.0,1.0)*options_.dt*options_.frameSkip,
                                         weapon->burstFromOriginBodyM))
            b.angularVelocityEcef={point->angularVelocity.x,point->angularVelocity.y,point->angularVelocity.z};
    const auto liveStates = pool_->states();
    std::vector<sim::VehicleState> states(liveStates.begin(), liveStates.end());
    if (std::isfinite(d.stepFraction)) {
        const double f=std::clamp(d.stepFraction,0.0,1.0);
        for(std::size_t slot=0;slot<states.size() && slot<stepStates_.size();++slot) {
            const auto* e=entries_[slot].get();
            if (e && ms::weapon::validMotionPath(e->engagementPath))
                states[slot]=engagementStateAt(*e,f*options_.dt*options_.frameSkip);
        }
    }
    struct DamageSnapshot {
        Entry* entry=nullptr;
        sim::VehicleState state;
        std::array<double,3> cg{};
        ms::damage::Target target;
    };
    const auto* sceneProvider=dynamic_cast<const ms::damage::WarheadSceneExecutionV2*>(adapter);
    const bool sharedScene=!adapter||sceneProvider;
    std::vector<DamageSnapshot> capturedVictims;
    std::vector<ms::damage::SceneVictimV1> sceneVictims;
    std::vector<ms::damage::SceneLiquidOnlyVictimV2> liquidOnlyVictims;
    std::vector<ms::damage::SceneLiquidOnlyWorkV2> liquidOnlyWork;
    ms::damage::SceneLiquidInputsV1 liquids;
    // Freeze the entire eligible set before any impulse/structural consequence
    // can create another body or remove geometry from the existing scene.
    for (std::size_t slot = 0; slot < states.size(); ++slot) {
        Entry* e = entries_[slot].get();
        if (!e || e == weapon || (e->info.role!=ms::Role::Aircraft&&e->info.role!=ms::Role::Debris)) continue;
        const auto af = airframes_.find(e->info.aircraft);
        const bool unsectioned=af==airframes_.end()&&sharedScene&&e->liquidMaterials&&!e->liquidMaterials->tanks.empty();
        if(af==airframes_.end()&&!unsectioned)continue;
        const sim::VehicleState& s = states[slot];
        const ms::damage::V3 at = v3(s.positionEcef);
        const double miss = ms::damage::length(b.at - at);
        // Retained external assess adapters keep their original dispatch
        // radius. A shared finite flight field uses actual material paths;
        // distance from its birth position is not a physical broad phase.
        if (!std::isfinite(miss)||(!sharedScene&&!(miss < kDamageReachM))) continue;
        auto& model = pool_->vehicle(slot);
        if(!unsectioned&&!ensureDamage(*e))continue;
        ms::damage::Target t;
        const double* R = s.rotationBodyToEcef;
        for (std::size_t k = 0; k < 3; ++k) t.bodyToEcef[k] = {R[k], R[3 + k], R[6 + k]};
        // (its model's origin, which its airframe's places are from: the state is at its CG)
        double cg[3];
        const bool liveCg=model.cgFromOriginBodyM(cg);
        // Reconstruct attached geometry and its impulse lever at the same
        // captured epoch as the encounter pose. Only a burst without an
        // explicit historical fraction requests the current native CG.
        if (std::isfinite(d.stepFraction)) {
            const auto captured=engagementCgAt(*e,std::clamp(d.stepFraction,0.0,1.0)*options_.dt*options_.frameSkip);
            if (!captured) {LOG_WARN("session") << "damage encounter CG unavailable for " << e->info.name;continue;}
            cg[0]=captured->x; cg[1]=captured->y; cg[2]=captured->z;
        } else if (!liveCg) continue;
        t.at = at - ms::damage::V3{R[0] * cg[0] + R[1] * cg[1] + R[2] * cg[2], R[3] * cg[0] + R[4] * cg[1] + R[5] * cg[2],
                                   R[6] * cg[0] + R[7] * cg[1] + R[8] * cg[2]};
        for (int r = 0; r < 3; ++r)
            (&t.velocity.x)[r] = R[3 * r] * s.velocityBodyMs[0] + R[3 * r + 1] * s.velocityBodyMs[1] + R[3 * r + 2] * s.velocityBodyMs[2];
        if (warhead->modelVersion==1 || warhead->modelVersion==2) {
            const auto origin=ms::weapon::materialPoint(s,v3(cg),{});
            if (!origin.valid) continue;
            t.at=origin.position; t.velocity=origin.velocity;
        }
        const auto matter=std::isfinite(d.stepFraction)
            ? engagementMatterAt(*e,std::clamp(d.stepFraction,0.0,1.0)*options_.dt*options_.frameSkip) : captureMatter(*e,model);
        if (!matter) { LOG_WARN("session") << "damage encounter atmosphere unavailable for " << e->info.name; continue; }
        t.airPressurePa=matter->pressurePa; t.airTemperatureK=matter->temperatureK; t.airDensity=matter->densityKgM3;
        t.angularVelocityEcef=rotate(s,v3(s.angularRateBodyRadS));
        std::string materialError;
        if(unsectioned) {
            if(!appendUnsectionedLiquidMaterialV2(*e,s,*matter,liquids,materialError)) {
                LOG_WARN("session") << "standalone liquid material refused for " << e->info.name << ": " << materialError;return out;
            }
            liquidOnlyVictims.push_back({e->info.id,e->info.generation,t,{}});
        } else {
            const auto attached=e->damage->attachedGeometry();
            if(sharedScene&&!appendLiquidMaterialV1(*e,s,*matter,attached,liquids,materialError)) {
                LOG_WARN("session") << "damage encounter liquid material refused for " << e->info.name << ": " << materialError;return out;
            }
            sceneVictims.push_back({e->info.id,attached,t,e->info.generation});
        }
        capturedVictims.push_back({e,s,{cg[0],cg[1],cg[2]},t});
        if (miss < nearest) nearest = miss, out[burstAt].target = static_cast<std::uint32_t>(slot) + 1;
    }
    if(causalRequested&&!sharedScene) {LOG_WARN("session") << "selected warhead provider has no causal scene capability";return {};}
    const bool causal=causalRequested&&sharedScene&&weapon&&weapon->info.role==ms::Role::Weapon&&std::isfinite(d.stepFraction);
    ms::damage::SceneAssessmentV1 assessment;
    if(sharedScene&&(!capturedVictims.empty()||causal)) {
        const auto ordinal=d.weapon?static_cast<std::uint64_t>(d.weapon):++scriptedDetonationSequence_;
        auto fragmentRng=Rng::forVehicle(options_.seed,kFragmentStream,ordinal);
        const auto field=sceneProvider?sceneProvider->emitScene(b,fragmentRng):ms::damage::emitFragmentFieldV1(*warhead,b,fragmentRng);
        ms::damage::SceneEnvironmentV1 environment;
        const auto closest=std::min_element(capturedVictims.begin(),capturedVictims.end(),[&](const auto& left,const auto& right) {
            const double a=ms::damage::length(left.target.at-b.at),c=ms::damage::length(right.target.at-b.at);
            return a<c||(a==c&&left.entry->info.id<right.entry->info.id);
        });
        // Scripted bursts have no native projectile atmosphere. Use the nearest
        // captured recipient's local native air as the declared frozen field.
        if(closest!=capturedVictims.end())environment.airDensity=closest->target.airDensity;
        if(weapon) {
            const auto matter=std::isfinite(d.stepFraction)?engagementMatterAt(*weapon,std::clamp(d.stepFraction,0.0,1.0)*options_.dt*options_.frameSkip):
                captureMatter(*weapon,pool_->vehicle(weapon->slot));
            if(matter)environment.airDensity=matter->densityKgM3;
        }
        const auto atmosphereSlot=weapon?weapon->slot:closest->entry->slot;
        if(const auto* source=dynamic_cast<const sim::NativePositionSourceV1*>(&pool_->vehicle(atmosphereSlot))) {
            sim::NativePositionV1 position;
            if(source->nativePositionV1(d.positionEcef,position)) {
                const double direction=units::degreesToRadians(environment_.windDirectionDeg);
                const double north=-environment_.windSpeedMs*std::cos(direction),east=-environment_.windSpeedMs*std::sin(direction);
                const double sl=std::sin(position.latitudeRad),cl=std::cos(position.latitudeRad),so=std::sin(position.longitudeRad),co=std::cos(position.longitudeRad);
                environment.airVelocity={-north*sl*co-east*so,-north*sl*so+east*co,north*cl};
                environment.gravityEcef={-kG*cl*co,-kG*cl*so,-kG*sl};
            }
        }
        if(causal) {
            if(!field.valid) {LOG_WARN("session") << "detonate: causal physical fragment source refused " << d.store;return out;}
            if(!damageContinuation_)damageContinuation_=std::make_shared<WorldDamageContinuationV1>();
            WorldDamageContinuationV1::Pending pending;
            pending.source=d;pending.correlation=weapon->origin;
            pending.sourceStreamSlot=static_cast<std::uint32_t>(weapon->slot);
            pending.sourceGeneration=weapon->info.generation;
            if(options_.publish)pending.diagnosticName=weapon->info.name;
            pending.diagnosticExplosiveKg=warhead->explosiveKg;
            pending.diagnosticTopology=static_cast<std::uint32_t>(warhead->fragmentSource);
            pending.correlation.id=weapon->info.id;pending.correlation.launcher=weapon->info.launcher;
            pending.correlation.store=d.store;pending.field=field;pending.environment=environment;
            const double dt=options_.dt*options_.frameSkip;
            pending.sampleTimeS=simTime_-dt*(1-std::clamp(d.stepFraction,0.0,1.0));
            pending.creationBoundaryS=simTime_;pending.lifetimeS=environment.maxFlightTimeS;
            captureWeaponFragmentDiagnosticsV1(d.weapon,pending.sourceGeneration,field,pending.sampleTimeS,
                pending.sampleTimeS,pending.diagnosticName,d.store,pending.diagnosticExplosiveKg,pending.diagnosticTopology);
            damageContinuation_->pending.push_back(std::move(pending));
            for(const auto& event:out)engage(event); // The burst is published once; future hits are not.
            if(contact)if(auto* live=entry(d.weapon))contactDamage(*live,*contact);
            return out;
        }
        if(liquids.recipients.empty())assessment=ms::damage::assessSceneV1(field,sceneVictims,environment);
        else {
            auto combined=ms::damage::assessSceneLiquidsV2(field,sceneVictims,liquidOnlyVictims,environment,liquids);
            assessment=std::move(combined.scene);liquidOnlyWork=std::move(combined.liquidOnlyWork);
        }
        if(!assessment.valid) {LOG_WARN("session") << "detonate: shared physical fragment assessment refused " << d.store;return out;}
        // The synchronous diagnostic retains the actual emitted source at
        // birth; its future assessed positions are not published as present.
        if(weapon&&weapon->info.role==ms::Role::Weapon) {
            const double epoch=std::isfinite(d.stepFraction)?simTime_-options_.dt*options_.frameSkip*(1-std::clamp(d.stepFraction,0.,1.)):simTime_;
            captureWeaponFragmentDiagnosticsV1(d.weapon,weapon->info.generation,field,epoch,epoch,
                weapon->info.name,d.store,warhead->explosiveKg,static_cast<std::uint32_t>(warhead->fragmentSource));
        }
    }
    for(const auto& captured:capturedVictims) {
        Entry* e=captured.entry;
        if(!e){LOG_WARN("session") << "damage consequence has no captured recipient";continue;}
        const auto& s=captured.state;const double* cg=captured.cg.data();const auto& t=captured.target;
        const auto af=airframes_.find(e->info.aircraft);
        if(af==airframes_.end()||!af->second) {
            if(sharedScene) {
                const auto row=std::find_if(assessment.victims.begin(),assessment.victims.end(),[&](const auto& value){return value.identity==e->info.id;});
                if(row!=assessment.victims.end()) {
                    captureLiquidAssessmentV1(*e,row->impact,d.weapon,weapon?weapon->info.generation:0,liquidOnlyWork,assessment.continuation);
                    const auto changes=applyDamage(*e,d,row->impact,s,cg,t.airPressurePa,{},true,true);
                    out.insert(out.end(),changes.begin(),changes.end());
                }
            }
            continue;
        }
        // applyDamage may create a first piece type and rehash airframes_.
        // Retain the actual shared frame, not an iterator across that call.
        const auto frame=af->second;
        ms::damage::Impact impact;
        if(sharedScene) {
            const auto row=std::find_if(assessment.victims.begin(),assessment.victims.end(),[&](const auto& value){return value.identity==e->info.id;});
            if(row==assessment.victims.end())continue;
            impact=row->impact;
        } else {
            // Existing external adapters retain their explicitly supplied assess
            // behavior; absence of the additive capability never selects a
            // different native physical model behind their vtable.
            impact=adapter->assess(b,e->damage->attachedGeometry(),t,e->damageRng);
        }
        std::vector<ms::damage::VehicleDamage::PointImpulse> contactLoads;
        if (contact && contact->victim==e->info.id) {
            const auto j=unrotate(s,contact->impulseEcef);
            impact.impulseBody+=j;
            impact.angularImpulseBody+=ms::damage::cross(contact->pointBody,j);
            contactLoads.push_back({contact->section,contact->pointBody,j});
        }
        const bool liquidAware=sharedScene&&std::any_of(liquids.recipients.begin(),liquids.recipients.end(),
            [&](const auto& value){return value.identity==e->info.id;});
        captureLiquidAssessmentV1(*e,impact,d.weapon,weapon?weapon->info.generation:0,liquidOnlyWork,assessment.continuation);
        const auto changes=applyDamage(*e,d,impact,s,cg,t.airPressurePa,contactLoads,sharedScene,liquidAware);
        out.insert(out.end(),changes.begin(),changes.end());
        LOG_INFO("session") << e->info.name << " hit by " << d.store << " at " << impact.missM << " m: " << impact.fragmentsStriking << " of "
                            << impact.fragmentsThrown << " fragments struck";
        for (std::size_t k = 0; k < frame->sections.size(); ++k)
            if (frame->sections[k].hasRoot)
                LOG_INFO("session") << "  its " << frame->sections[k].name << ": strength left " << e->damage->sections()[k].strength
                                    << ", the blast's and the fragments' kick " << e->damage->sections()[k].kick << " of it";
    }
    for (const auto& ev : out) engage(ev);
    return out;
}

bool World::pendingDetonationEffects() const noexcept {
    return damageContinuation_&&!damageContinuation_->pending.empty();
}

void World::advanceDetonationEffects() {
    if(!pendingDetonationEffects())return;
    const double nativeDuration=options_.dt*options_.frameSkip;
    struct Snapshot {Entry* entry=nullptr;sim::VehicleState state;std::array<double,3> cg{};ms::damage::Target target;};
    auto& pending=damageContinuation_->pending;
    for(auto& effect:pending) {
        if(effect.lastPublicationS==simTime_)continue;
        const double offset=effect.creationBoundaryS==simTime_?
            std::clamp(effect.source.stepFraction,0.0,1.0)*nativeDuration:0;
        const double duration=std::min(nativeDuration-offset,effect.lifetimeS-effect.field.ageS);
        effect.lastPublicationS=simTime_;
        if(!(duration>0))continue;
        const double endOffset=std::min(nativeDuration,offset+duration);
        std::vector<Snapshot> snapshots;std::vector<ms::damage::SceneVictimV1> victims;
        std::vector<ms::damage::SceneLiquidOnlyVictimV2> liquidOnlyVictims;
        ms::damage::SceneLiquidInputsV1 liquids;
        bool materialAdmitted=true;
        // Borrowed Entry pointers exist only inside this serialized native
        // operation. The retained field owns immutable IDs/generations instead.
        for(auto& owned:entries_)if(auto* e=owned.get();e&&(e->info.role==ms::Role::Aircraft||e->info.role==ms::Role::Debris)) {
            const bool structural=ensureDamage(*e);
            const bool unsectioned=!structural&&e->liquidMaterials&&!e->liquidMaterials->tanks.empty();
            if(e->info.id==effect.correlation.id||(!structural&&!unsectioned)||!ms::weapon::validMotionPath(e->engagementPath)||
               e->engagementPath.front().offsetS>offset||e->engagementPath.back().offsetS<endOffset)continue;
            const auto initial=engagementStateAt(*e,offset);
            ms::damage::SceneLiquidOnlyVictimV2 victim{e->info.id,e->info.generation,{},{}};
            std::array<double,3> firstCg{};bool admitted=true;
            for(double local=0;;local=std::min(duration,local+effect.environment.maxSubstepS)) {
                const double sampleOffset=local==duration?endOffset:offset+local;
                const auto state=engagementStateAt(*e,sampleOffset);
                const auto cg=engagementCgAt(*e,sampleOffset);const auto matter=engagementMatterAt(*e,sampleOffset);
                if(!cg||!matter){admitted=false;break;}
                const auto origin=ms::weapon::materialPoint(state,*cg,{});if(!origin.valid){admitted=false;break;}
                ms::damage::Target target;target.at=origin.position;target.velocity=origin.velocity;
                for(std::size_t axis=0;axis<3;++axis)
                    target.bodyToEcef[axis]={state.rotationBodyToEcef[axis],state.rotationBodyToEcef[axis+3],state.rotationBodyToEcef[axis+6]};
                target.airPressurePa=matter->pressurePa;target.airTemperatureK=matter->temperatureK;target.airDensity=matter->densityKgM3;
                target.angularVelocityEcef=rotate(state,v3(state.angularRateBodyRadS));
                if(local==0){victim.target=target;firstCg={cg->x,cg->y,cg->z};}
                victim.motion.push_back({local,target});if(local==duration)break;
            }
            if(admitted) {
                const auto matter=engagementMatterAt(*e,offset);
                std::string materialError;
                const bool material=bool(matter)&&(unsectioned?appendUnsectionedLiquidMaterialV2(*e,initial,*matter,liquids,materialError):
                    appendLiquidMaterialV1(*e,initial,*matter,e->damage->attachedGeometry(),liquids,materialError));
                if(!material) {
                    LOG_WARN("session") << "causal fragment liquid material refused for " << e->info.name << ": " << materialError;
                    materialAdmitted=false;break;
                }
                snapshots.push_back({e,initial,firstCg,victim.target});
                if(unsectioned)liquidOnlyVictims.push_back(std::move(victim));
                else victims.push_back({e->info.id,e->damage->attachedGeometry(),victim.target,e->info.generation,std::move(victim.motion)});
            }
        }
        auto environment=effect.environment;environment.maxFlightTimeS=duration;
        if(!materialAdmitted){effect.field.projectiles.clear();continue;}
        ms::damage::SceneLiquidAssessmentV2 combined;
        if(liquids.recipients.empty())combined.scene=ms::damage::assessSceneV1(effect.field,victims,environment);
        else combined=ms::damage::assessSceneLiquidsV2(effect.field,victims,liquidOnlyVictims,environment,liquids);
        const auto& result=combined.scene;
        if(!result.valid) {
            LOG_WARN("session") << "causal fragment continuation refused weapon=" << effect.correlation.id;
            effect.field.projectiles.clear();continue;
        }
        for(const auto& snapshot:snapshots) {
            const auto row=std::find_if(result.victims.begin(),result.victims.end(),[&](const auto& impact){return impact.identity==snapshot.entry->info.id;});
            if(row==result.victims.end())continue;
            captureLiquidAssessmentV1(*snapshot.entry,row->impact,effect.correlation.id,effect.sourceGeneration,combined.liquidOnlyWork,result.continuation);
            Detonation cause=effect.source;cause.stepFraction=std::numeric_limits<double>::quiet_NaN();
            const auto changes=applyDamageWithSource(*snapshot.entry,cause,row->impact,snapshot.state,snapshot.cg.data(),
                snapshot.target.airPressurePa,{},effect.sourceStreamSlot,effect.sourceGeneration,true,
                std::any_of(liquids.recipients.begin(),liquids.recipients.end(),[&](const auto& value){return value.identity==snapshot.entry->info.id;}));
            for(const auto& event:changes)engage(event);
            if(!changes.empty()) {
                ms::EngagementEvent event;event.serial=nextCombatSerial_++;event.timeS=simTime_;
                event.kind="damage";event.reason="warhead";event.entity=effect.correlation.id;
                event.launcher=effect.correlation.launcher;event.target=snapshot.entry->info.id;event.destroyed=snapshot.entry->crewLost;
                std::copy_n(effect.source.positionEcef,3,event.positionEcef);std::copy_n(effect.source.velocityEcef,3,event.velocityEcef);
                recordWeaponEventGenerationV1(event,effect.sourceGeneration,snapshot.entry->info.generation);
                combatEvents_.push_back(std::move(event));
            }
        }
        effect.field=result.continuation;
        const double capturedEpoch=simTime_-(nativeDuration-endOffset);
        captureWeaponFragmentDiagnosticsV1(effect.correlation.id,effect.sourceGeneration,effect.field,capturedEpoch,
            effect.sampleTimeS,effect.diagnosticName,effect.source.store,effect.diagnosticExplosiveKg,effect.diagnosticTopology);
    }
    pending.erase(std::remove_if(pending.begin(),pending.end(),[&](const auto& effect) {
        return effect.field.projectiles.empty()||effect.field.ageS>=effect.lifetimeS;
    }),pending.end());
}

void World::stepDamage() {
    advanceDetonationEffects();
    const double dt = options_.dt * options_.frameSkip;
    // a ground strike's fire, burnt out
    for (auto& entry : entries_)
        if (Entry* e = entry.get(); e && e->wreckFireUntilS > 0.0 && simTime_ >= e->wreckFireUntilS) {
            engage(engagementEvent(*e, ipc::EngagementKind::Fire, e->wreckFireName)); // (its rate 0: it is out)
            e->wreckFireUntilS = 0.0;
        }
    for (std::size_t slot = 0; slot < entries_.size(); ++slot) {
        Entry* e = entries_[slot].get();
        if (!e || !e->damage) continue;
        auto& model = pool_->vehicle(slot);
        (void)model;
        e->working = pool_->states()[slot];
        ms::damage::Flight f;
        damageFlight(*e, f);
        // the holed tanks leak; a burning one's rate recorded as it changes, its end when it is dry
        for (const auto& [tank, state] : e->damage->tanks()) {
            if (tank < 0 || static_cast<std::size_t>(tank) >= f.tankKg.size()) continue;
            const double kg = f.tankKg[static_cast<std::size_t>(tank)];
            const double leak = std::min(e->damage->leakKgS(tank, kg, e->working.loadFactor, f.dynamicPressurePa),kg/dt);
            if(leak>0) {
                if(e->damage->detachedFuelKg(tank))e->damage->removeDetachedFuelKg(tank,leak*dt);
                else if(static_cast<std::size_t>(tank)<e->damageReads.tanks.size())
                    e->damageReads.tanks[static_cast<std::size_t>(tank)].set(std::max(kg-leak*dt,0.0)/kLbsToKg);
            }
            if (!state.burning) continue;
            const auto last = e->fireRates.find(tank);
            const double before = last == e->fireRates.end() ? 0.0 : last->second;
            if (std::abs(leak - before) > kRateReported * std::max(before, 1e-3) && !(before == 0.0 && leak == 0.0)) {
                const auto& comp = e->damage->airframe().components[static_cast<std::size_t>(std::max(state.component, 0))];
                auto ev = engagementEvent(*e, ipc::EngagementKind::Fire, comp.name);
                double cg[3];
                if(!model.cgFromOriginBodyM(cg)) {LOG_WARN("session") << "fire component CG unavailable for " << e->info.name;continue;}
                const auto point=ms::weapon::materialPoint(e->working,v3(cg),comp.centre);
                if(!point.valid) {LOG_WARN("session") << "fire component point unavailable for " << e->info.name;continue;}
                for(int k=0;k<3;++k) {
                    ev.positionEcef[k]=(&point.position.x)[k];
                    ev.velocityEcef[k]=(&point.velocity.x)[k];
                }
                ev.massKg = leak;
                ev.rateKgS = std::min(leak, ms::damage::kBurnMostKgS);
                engage(ev);
                e->fireRates[tank] = leak;
            }
        }
        if(e->info.role==ms::Role::Debris) {
            synchronizeDetachedFuel(model,*e->damage);
        }
        // each section's root against its strength left: one that fails leaves, with its mass and its tanks' fuel
        for (const auto& c : e->damage->loads(f)) separated(*e, c.section);
    }
}

bool World::ensureDamage(Entry& e) {
    if (e.damage) return true;
    const auto af = airframes_.find(e.info.aircraft);
    if (af == airframes_.end() || (e.info.role!=ms::Role::Aircraft&&
        !(e.info.role==ms::Role::Debris&&e.damageOwnedSection))) return false;
    auto& model = pool_->vehicle(e.slot);
    e.damage=e.damageOwnedSection?std::make_unique<ms::damage::VehicleDamage>(af->second,*e.damageOwnedSection):
        std::make_unique<ms::damage::VehicleDamage>(af->second);
    if(e.damageOwnedSection)for(const auto& tank:af->second->sections[static_cast<std::size_t>(*e.damageOwnedSection)].tanks)
        e.damage->setDetachedFuelKg(tank.index,0); // Native physical reset restores the empty piece model.
    e.damageRng = Rng::forVehicle(options_.seed, kDamageStream, e.info.id);
    auto& r = e.damageReads;
    const char* axes[3] = {"x", "y", "z"};
    for (std::size_t k = 0; k < 3; ++k) {
        r.aero[k] = model.property(std::string("forces/fb") + axes[k] + "-aero-lbs");
        r.total[k] = model.property(std::string("forces/fb") + axes[k] + "-total-lbs");
    }
    r.weight = model.property("inertia/weight-lbs");
    r.qbar = model.property("aero/qbar-psf");
    int tanks = 0; // (its tanks: as many as it lists - the first's name has no index)
    for (const auto& [name, value] : model.properties("propulsion")) { // (names below the prefix)
        (void)value;
        if (name == "tank/contents-lbs") tanks = std::max(tanks, 1);
        const auto close = name.find(']');
        if (name.rfind("tank[", 0) == 0 && close != std::string::npos && name.compare(close, std::string::npos, "]/contents-lbs") == 0)
            tanks = std::max(tanks, std::stoi(name.substr(5, close - 5)) + 1);
    }
    for (int k = 0; k < tanks; ++k) r.tanks.push_back(model.property("propulsion/tank[" + std::to_string(k) + "]/contents-lbs"));
    return true;
}

void World::separated(Entry& e, int section, const sim::VehicleState* as, const Thrown* thrown) {
    separatedAt(e,section,as,thrown,nullptr);
}

void World::separatedAt(Entry& e, int section, const sim::VehicleState* as, const Thrown* thrown,
                         std::vector<ipc::EngagementEvent>* events) {
    const auto& a = e.damage->airframe();
    const auto& sec = a.sections[static_cast<std::size_t>(section)];
    auto& model = pool_->vehicle(e.slot);
    // the aircraft as it was, and where its CG is from its model's origin (the section's place is from that)
    Parting part;
    if (as) part.before=*as;
    else model.state(part.before);
    model.cgFromOriginBodyM(part.cgFromOrigin);
    // Copy accumulated holes, fire and component state before clearing parent
    // tank contents. The piece's empty native origin is this section's CG.
    auto inherited=e.damage->detachedState(section,sec.cg);
    // what leaves with it: its structure, its tanks' fuel, its stations' stores - off the aircraft at once, so its CG,
    // its inertia and its state move to what is left (FlightModel::endMassChange)
    model.beginMassChange();
    double at[3];
    structural(a, sec.cg, at);
    model.removeMass(sec.massKg, at);
    for (const auto& t : sec.tanks)
        if (t.index >= 0 && static_cast<std::size_t>(t.index) < e.damageReads.tanks.size()) {
            auto& contents = e.damageReads.tanks[static_cast<std::size_t>(t.index)];
            const double kg=contents.get()*kLbsToKg;part.fuelKg+=kg;
            if(inherited)inherited->setDetachedFuelKg(t.index,kg);
            contents.set(0.0);
        }
    for (const int station : sec.stations) part.stations.emplace_back(station, e.ms.loseStation(station, simTime_));
    applyStoreMasses(e);
    model.endMassChange(sec.inertia.data(), nullptr);
    // Load the real departing model before applying the internal impulse. Fuel
    // and stores move its CG away from the dry section centre; both bodies must
    // receive their equal/opposite impulses at that same final loaded point.
    const auto pieceId=launchPiece(e,section,part);
    if(auto* piece=entry(pieceId);piece&&inherited) {
        piece->damageOwnedSection=section;
        piece->damage=std::move(inherited);piece->damageRng=Rng::forVehicle(options_.seed,kDamageStream,pieceId);
        piece->liquidMaterials=e.liquidMaterials;
        piece->liquidOriginShiftBodyM=e.liquidOriginShiftBodyM+sec.cg;
        piece->crewLost=piece->damage->crewLost();
        airframes_[piece->info.aircraft]=std::make_shared<ms::damage::Airframe>(piece->damage->airframe());
        auto& pieceModel=pool_->vehicle(piece->slot);auto& reads=piece->damageReads;
        const char* axes[]={"x","y","z"};
        for(std::size_t i=0;i<3;++i) {
            reads.aero[i]=pieceModel.property(std::string("forces/fb")+axes[i]+"-aero-lbs");
            reads.total[i]=pieceModel.property(std::string("forces/fb")+axes[i]+"-total-lbs");
        }
        reads.weight=pieceModel.property("inertia/weight-lbs");reads.qbar=pieceModel.property("aero/qbar-psf");
        synchronizeDetachedFuel(pieceModel,*piece->damage);
        // The parent no longer owns this emitter. The child's next physical
        // leak update publishes the inherited fire at its translated component.
        for(const auto& [tank,state]:piece->damage->tanks())if(state.burning) {
            const auto component=static_cast<std::size_t>(std::max(state.component,0));
            if(component<a.components.size()) {
                const auto stop=engagementEvent(e,ipc::EngagementKind::Fire,a.components[component].name);
                if(events)events->push_back(stop);else engage(stop);
            }
            e.fireRates[tank]=0;
        }
    }
    if (const auto* piece=entry(pieceId)) {
        auto& pieceModel=pool_->vehicle(piece->slot);
        auto& restModel=pool_->vehicle(e.slot);
        sim::VehicleState pieceState,restState;
        pieceModel.state(pieceState); restModel.state(restState);
        const double partKg=pieceModel.energy().massKg;
        if (std::isfinite(partKg) && partKg>0) {
            const auto loadedArm=unrotate(part.before,v3(pieceState.positionEcef)-v3(part.before.positionEcef));
            const auto loadedCg=v3(part.cgFromOrigin)+loadedArm;
            ms::damage::V3 push=ms::damage::normalized(sec.cg-sec.root)*kSeparationMs;
            if (thrown) {
                const auto referenceCg=thrown->sharedCgFromOriginBodyM.value_or(v3(part.cgFromOrigin));
                auto extra=v3(thrown->impulseBody)*(1/partKg)-v3(thrown->sharedBody)-
                    ms::damage::cross(v3(thrown->sharedAngularBody),loadedCg-referenceCg);
                if (const double speed=ms::damage::length(extra); speed>kThrownMostMs) extra=extra*(kThrownMostMs/speed);
                push+=extra;
            }
            const auto impulseEcef=rotate(part.before,push*partKg);
            const auto pieceImpulse=unrotate(pieceState,impulseEcef);
            const auto restImpulse=unrotate(restState,impulseEcef*(-1));
            const auto restArm=unrotate(restState,v3(pieceState.positionEcef)-v3(restState.positionEcef));
            const auto restAngular=ms::damage::cross(restArm,restImpulse);
            const double pieceNs[]={pieceImpulse.x,pieceImpulse.y,pieceImpulse.z};
            const double restNs[]={restImpulse.x,restImpulse.y,restImpulse.z};
            const double restNms[]={restAngular.x,restAngular.y,restAngular.z};
            pieceModel.applyImpulse(pieceNs,nullptr);
            restModel.applyImpulse(restNs,restNms);
            pool_->refreshState(piece->slot);
        } else LOG_WARN("session") << e.info.name << ": departing section mass unavailable for its internal impulse";
    }
    const auto emit=[&](ipc::EngagementEvent event){ if (events) events->push_back(event); else engage(event); };
    for (const auto& change : e.damage->detachComponents(section)) {
        const auto& component=a.components[static_cast<std::size_t>(change.component)];
        using K=ms::damage::Change::Kind;
        if (change.kind==K::CrewLost) {
            e.crewLost=true; emit(engagementEvent(e,ipc::EngagementKind::CrewLost,component.name));
        } else if (change.kind==K::EngineOut) {
            model.seizeEngine(change.engine); emit(engagementEvent(e,ipc::EngagementKind::EngineOut,component.name));
        } else if (change.kind==K::ControlLost) {
            model.failChannel(change.channel); emit(engagementEvent(e,ipc::EngagementKind::ControlLost,change.channel));
        }
    }
    emit(engagementEvent(e, ipc::EngagementKind::Separated, sec.name));
    pool_->refreshState(e.slot);
    LOG_INFO("session") << e.info.name << ": its " << sec.name << " failed at its root and left";
}

bool World::separateSection(std::uint32_t id, const std::string& section) {
    Entry* e = entry(id);
    if (!e || !ensureDamage(*e)) return false;
    const int s = e->damage->airframe().section(section);
    if (s < 0 || !e->damage->separate(s)) return false;
    separated(*e, s);
    return true;
}

std::uint32_t World::launchPiece(Entry& from, int section, const Parting& part) {
    const auto& a = from.damage->airframe();
    const auto& sec = a.sections[static_cast<std::size_t>(section)];
    const auto dir = aircraftDirs_.find(from.info.aircraft);
    std::string slug = sec.name;
    for (char& c : slug)
        if (c == ' ') c = '-';
    const std::string piece = from.info.aircraft + "-" + slug;
    if (dir == aircraftDirs_.end() || !std::filesystem::is_regular_file(dir->second / "pieces" / piece / (piece + ".xml"))) {
        // (it has left all the same - its mass, fuel and stores off the aircraft - flown by nothing)
        LOG_WARN("session") << from.info.name << ": no flight model for its " << sec.name << " (hangar's pieces/" << piece << ")";
        return 0;
    }
    const sim::VehicleState& s = part.before;
    const double* R = s.rotationBodyToEcef;
    auto toEcef = [&](const ms::damage::V3& b) {
        return ms::damage::V3{R[0] * b.x + R[1] * b.y + R[2] * b.z, R[3] * b.x + R[4] * b.y + R[5] * b.z, R[6] * b.x + R[7] * b.y + R[8] * b.z};
    };
    // where its centre was (from the aircraft's CG, which the state is at: the section's place is from the model's
    // origin), as it moved there (the aircraft's velocity and its rotation's), pushed off its root
    const ms::damage::V3 r = sec.cg - v3(part.cgFromOrigin);
    const ms::damage::V3 at = v3(s.positionEcef) + toEcef(r);
    const ms::damage::V3 rate = v3(s.angularRateBodyRadS);
    const ms::damage::V3 dv = toEcef(ms::damage::cross(rate, r) + v3(part.pushBody));
    const auto* coordinateSource=dynamic_cast<const sim::NativePositionSourceV1*>(&pool_->vehicle(from.slot));
    sim::NativePositionV1 position;
    const double ecef[]={at.x,at.y,at.z};
    if (!coordinateSource || !coordinateSource->nativePositionV1(ecef,position)) {
        LOG_WARN("session") << from.info.name << ": native position conversion unavailable for its " << sec.name;
        return 0;
    }
    const double lat=position.latitudeRad,lon=position.longitudeRad;
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    const ms::damage::V3 north{-sl * co, -sl * so, cl}, east{-so, co, 0.0}, down{-cl * co, -cl * so, -sl};
    VehicleSpec spec;
    spec.name = from.info.name + " " + sec.name;
    spec.type = "jsbsim:" + piece;
    spec.aircraftDir = dir->second / "pieces";
    spec.model = from.info.model;
    spec.initial.latitudeDeg = units::radiansToDegrees(lat);
    spec.initial.longitudeDeg = units::radiansToDegrees(lon);
    spec.initial.altitudeMslM=position.altitudeMslM;
    // Local NED axes change at the piece's position. Preserve the inherited
    // inertial attitude and velocity instead of copying the old local values.
    const ms::damage::V3 bx{R[0],R[3],R[6]},by{R[1],R[4],R[7]},bz{R[2],R[5],R[8]};
    spec.initial.rollDeg=units::radiansToDegrees(std::atan2(ms::damage::dot(down,by),ms::damage::dot(down,bz)));
    spec.initial.pitchDeg=units::radiansToDegrees(std::asin(std::clamp(-ms::damage::dot(down,bx),-1.,1.)));
    spec.initial.headingDeg=units::radiansToDegrees(std::atan2(ms::damage::dot(east,bx),ms::damage::dot(north,bx)));
    const auto velocityEcef=toEcef(v3(s.velocityBodyMs))+dv;
    spec.initial.velocityNedMs[0]=ms::damage::dot(velocityEcef,north);
    spec.initial.velocityNedMs[1]=ms::damage::dot(velocityEcef,east);
    spec.initial.velocityNedMs[2]=ms::damage::dot(velocityEcef,down);
    spec.initial.airspeedTrueMs = s.airspeedTrueMs;
    for (int k = 0; k < 3; ++k) spec.initial.bodyRatesRadS[k] = s.angularRateBodyRadS[k];
    const std::uint32_t id = createVehicle(spec);
    Entry* p = entry(id);
    if (!p) return 0;
    p->info.role = ms::Role::Debris;
    p->info.launcher = from.info.id;
    p->info.launchTimeS = simTime_;
    for (std::size_t c = 0; c < p->catalog->size(); ++c)
        p->host.setAvailability(c, control::Availability::Unavailable, control::Reason::Restricted);
    p->inputs = sim::ControlInputs{};
    poolInputs_[p->slot] = p->inputs;
    // what it took with it: its tanks' fuel (at their middle), each station's stores (at its place) - its own CG and
    // state then moved to all of it, as the aircraft's were (each part moving as its point of the aircraft did)
    auto& model = pool_->vehicle(p->slot);
    model.beginMassChange();
    model.property("inertia/pointmass-weight-lbs").set(part.fuelKg / kLbsToKg); // ("fuel")
    double storesKg = 0.0;
    for (std::size_t k = 0; k < part.stations.size(); ++k) {
        model.property("inertia/pointmass-weight-lbs[" + std::to_string(k + 1) + "]").set(part.stations[k].second / kLbsToKg);
        storesKg += part.stations[k].second;
    }
    model.endMassChange(nullptr, nullptr);
    // drawn as its aircraft's model, its section alone: the model's origin from the piece's CG (its structure's
    // centre and what it carries)
    double shift[3];
    model.cgFromOriginBodyM(shift);
    char offset[96];
    std::snprintf(offset, sizeof offset, ":%.4f,%.4f,%.4f", -(sec.cg.x + shift[0]), -(sec.cg.y + shift[1]), -(sec.cg.z + shift[2]));
    p->info.type = "debris:" + from.info.aircraft + ":" + sec.name + offset;
    publishVehicle(*p); // (recorded, and its row told to a live viewer: its type is what it is drawn by)
    LOG_INFO("session") << "piece " << id << " '" << spec.name << "': " << sec.massKg + part.fuelKg + storesKg << " kg (" << part.fuelKg
                        << " of fuel, " << storesKg << " of stores)";
    return id;
}

void World::impacted(Entry& e, const sim::VehicleState& fallback, const sim::VehicleState& after) {
    namespace dmg = ms::damage;
    auto& model = pool_->vehicle(e.slot);
    sim::VehicleState before = fallback;
    sim::TerrainStrikeSnapshotV1 incoming;
    const auto* source = dynamic_cast<const sim::TerrainStrikeSourceV1*>(&model);
    const bool captured = source && source->terrainStrikeSnapshotV1(incoming);
    if (captured) before = incoming.state;
    // the local axes there (ECEF), and the ground's normal: its slope over its data's spacing
    const double lat = after.latitudeRad, lon = after.longitudeRad;
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    const dmg::V3 north{-sl * co, -sl * so, cl}, east{-so, co, 0.0}, down{-cl * co, -cl * so, -sl};
    dmg::V3 normal = down * -1.0;
    if (ground_ && std::isfinite(ground_->resolutionM())) {
        const double h = std::clamp(ground_->resolutionM(), 5.0, 60.0);
        const double dn = h / 6371000.0, de = h / (6371000.0 * std::max(cl, 1e-6));
        const double slopeN = (ground_->heightAboveEllipsoidM(lat + dn, lon) - ground_->heightAboveEllipsoidM(lat - dn, lon)) / (2.0 * h);
        const double slopeE = (ground_->heightAboveEllipsoidM(lat, lon + de) - ground_->heightAboveEllipsoidM(lat, lon - de)) / (2.0 * h);
        if (std::isfinite(slopeN) && std::isfinite(slopeE)) normal = dmg::normalized(north * -slopeN + east * -slopeE + down * -1.0);
    }
    // Incoming NED axes belong to that sample's position, not the stopped CG.
    const double bs = std::sin(before.latitudeRad), bc = std::cos(before.latitudeRad);
    const double bo = std::sin(before.longitudeRad), bco = std::cos(before.longitudeRad);
    const dmg::V3 beforeNorth{-bs * bco, -bs * bo, bc}, beforeEast{-bo, bco, 0.0}, beforeDown{-bc * bco, -bc * bo, -bs};
    auto ecef = [&](const double ned[3]) { return beforeNorth * ned[0] + beforeEast * ned[1] + beforeDown * ned[2]; };
    dmg::GroundStrike strike;
    strike.velocity = ecef(before.velocityNedMs);
    strike.normal = normal;
    const sim::EnergyOnBoard aboard = model.energy();
    const double impactMass = captured ? incoming.massKg : aboard.massKg;
    strike.massKg = std::isfinite(impactMass) ? impactMass : 0.0;
    // Fuel describes what remains when this native strike is applied. Fuel
    // already consumed/leaked during contact cannot burn a second time.
    strike.fuelKg = std::isfinite(aboard.fuelKg) ? std::max(aboard.fuelKg, 0.0) : 0.0;
    // a store that was released ends where it strikes (World removes it): nothing of it burns on. A tank's fuel is the
    // point mass its flight model names "fuel" (hangar's storemodels.py), set as it left its station
    const bool stays = e.info.role != ms::Role::Weapon;
    if (!stays)
        if (const auto fuel = model.property("inertia/pointmass-weight-lbs"); fuel.valid()) strike.fuelKg = std::max(fuel.get() * kLbsToKg, 0.0);
    const dmg::StrikeOutcome o = dmg::groundStrike(strike);

    // its sections against the strike's load, as against a flight's: what an accelerometer read through the crush, in
    // its body axes as it struck - a root that cannot hold its section against that fails, and the section goes on
    // with what the strike left it (the wreck itself is stopped where it lies)
    int broke = 0;
    if (o.normalMs > 0.0 && e.info.role == ms::Role::Aircraft && ensureDamage(e)) {
        const double* R = before.rotationBodyToEcef; // (row-major, columns the body axes: its transpose turns ECEF to body)
        const dmg::V3 f = o.specificForce;
        dmg::Flight flight;
        flight.specificForceMs2 = {R[0] * f.x + R[3] * f.y + R[6] * f.z, R[1] * f.x + R[4] * f.y + R[7] * f.z, R[2] * f.x + R[5] * f.y + R[8] * f.z};
        flight.rateRadS = v3(before.angularRateBodyRadS);
        flight.tankKg.resize(e.damageReads.tanks.size());
        for (std::size_t t = 0; t < e.damageReads.tanks.size(); ++t) flight.tankKg[t] = e.damageReads.tanks[t].get() * kLbsToKg;
        // (thrown from where it lies, a little clear of the ground, as it flew in but with the strike's velocity)
        sim::VehicleState thrown = before;
        const double clear = 0.5 * o.crushM;
        for (int k = 0; k < 3; ++k) thrown.positionEcef[k] = after.positionEcef[k] + (&normal.x)[k] * clear;
        thrown.latitudeRad = after.latitudeRad, thrown.longitudeRad = after.longitudeRad;
        thrown.velocityNedMs[0] = dmg::dot(o.after, north), thrown.velocityNedMs[1] = dmg::dot(o.after, east), thrown.velocityNedMs[2] = dmg::dot(o.after, down);
        // launchPiece consumes body velocity. Keep every representation of
        // this same post-impact vector consistent; retaining the incoming
        // body velocity discards the native rebound and immediately drives
        // detached pieces back into the terrain.
        for (int k = 0; k < 3; ++k)
            thrown.velocityBodyMs[k] = R[k] * o.after.x + R[3 + k] * o.after.y + R[6 + k] * o.after.z;
        thrown.airspeedTrueMs = dmg::length(o.after);
        for (const auto& c : e.damage->loads(flight)) {
            separated(e, c.section, &thrown);
            ++broke;
        }
    }

    ipc::EngagementEvent ev = engagementEvent(e, ipc::EngagementKind::Impact, "ground");
    for (int k = 0; k < 3; ++k) {
        ev.velocityEcef[k] = (&strike.velocity.x)[k];
        ev.normalEcef[k] = (&normal.x)[k];
    }
    ev.massKg = strike.massKg;
    ev.energyJ = o.energyJ;
    ev.angleRad = o.angleRad;
    ev.loadG = o.decelMs2 / kG;
    ev.index = static_cast<std::uint32_t>(broke);
    ev.amountKg = o.fireballKg;
    ev.rateKgS = stays ? std::min(o.poolKgS, dmg::kBurnMostKgS) : 0.0;
    engage(ev);
    LOG_INFO("session") << e.info.name << " struck the ground at " << o.speedMs << " m/s, " << o.angleRad * 57.29577951308232
                        << " deg below it: " << ev.loadG << " g across it, " << broke << " section(s) broke off, " << o.fireballKg
                        << " kg of fuel burnt at once, " << o.poolKg << " kg burns on";

    // its fuel's fire: the mist's fireball, moving as what left the strike does, and what burns on where it lies - from
    // a tank of its own where its damage data names one (the viewer hangs the flame there)
    if (o.fireballKg > 0.0 || ev.rateKgS > 0.0) {
        std::string component = "fuel";
        if (e.damage) {
            for (const auto& c : e.damage->airframe().components)
                if (c.kind == "fuel" && (c.section < 0 || !e.damage->sections()[static_cast<std::size_t>(c.section)].separated)) {
                    component = c.name;
                    break;
                }
        } else if (e.info.type.rfind("debris:", 0) == 0) {
            // a piece (debris:<aircraft>:<section>:<offset>): a tank of its section, as its aircraft's damage data has it
            const auto second = e.info.type.find(':', 7);
            const auto third = second == std::string::npos ? second : e.info.type.find(':', second + 1);
            const auto af = second == std::string::npos ? airframes_.end() : airframes_.find(e.info.type.substr(7, second - 7));
            if (af != airframes_.end() && third != std::string::npos) {
                const std::string section = e.info.type.substr(second + 1, third - second - 1);
                for (const auto& c : af->second->components)
                    if (c.kind == "fuel" && c.section >= 0 && af->second->sections[static_cast<std::size_t>(c.section)].name == section) {
                        component = c.name;
                        break;
                    }
            }
        }
        ipc::EngagementEvent fire = engagementEvent(e, ipc::EngagementKind::Fire, component);
        for (int k = 0; k < 3; ++k) {
            fire.velocityEcef[k] = (&o.after.x)[k];
            fire.normalEcef[k] = (&normal.x)[k]; // (on the ground: the viewer's fireball a hemisphere, rolling on along it)
        }
        fire.amountKg = o.fireballKg;
        fire.rateKgS = ev.rateKgS;
        fire.massKg = ev.rateKgS; // (all of it burns: nothing blows off a wreck as mist)
        engage(fire);
        if (ev.rateKgS > 0.0) {
            e.wreckFireName = component;
            e.wreckFireUntilS = simTime_ + o.poolKg / ev.rateKgS;
        }
    }
}

void World::engage(const ipc::EngagementEvent& event) {
    engagement_.push_back(event);
    if (publisher_) publisher_->publishEvent(event); // (a live viewer draws it as it happens)
    // (recorded as it happens: the reader gives it to the frame that follows - G7)
    if (recorder_.isOpen()) recorder_.writeEvent(event);
}

std::vector<ipc::EngagementEvent> World::takeEngagementEvents() {
    std::vector<ipc::EngagementEvent> out;
    out.swap(engagement_);
    return out;
}

std::optional<std::uint32_t> World::vehicleStreamSlot(std::uint32_t id) const noexcept {
    const auto* e=entry(id);
    if(!e || !e->info.alive)return std::nullopt;
    return static_cast<std::uint32_t>(e->slot);
}

const ms::damage::VehicleDamage* World::damage(std::uint32_t id) const noexcept {
    const Entry* e = entry(id);
    return e ? e->damage.get() : nullptr;
}

} // namespace fsim::session
