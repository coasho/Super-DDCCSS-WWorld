#include "World.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::session {
void World::pruneWeaponEffectsDiagnosticsV1() {
    for(auto it=weaponEffectsDiagnosticsV1_.begin();it!=weaponEffectsDiagnosticsV1_.end();) {
        if(simTime_>it->second.retainedUntilS)it=weaponEffectsDiagnosticsV1_.erase(it);
        else ++it;
    }
}
void World::captureWeaponFragmentDiagnosticsV1(std::uint32_t weapon,std::uint64_t generation,
    const ms::damage::FragmentFieldV1& field,double epoch,double burstEpoch,
    std::string_view name,std::string_view store,double explosiveKg,std::uint32_t topology) {
    if(!options_.publish||!weapon||!generation||!field.valid||!std::isfinite(epoch)||
       !std::isfinite(burstEpoch)||burstEpoch<0||burstEpoch>epoch||epoch>simTime_||
       !std::isfinite(explosiveKg)||explosiveKg<0)return;
    pruneWeaponEffectsDiagnosticsV1();
    const auto key=std::make_pair(weapon,generation);
    auto found=weaponEffectsDiagnosticsV1_.find(key);
    if(found==weaponEffectsDiagnosticsV1_.end()) {
        if(weaponEffectsDiagnosticsV1_.size()>=128) {
            const auto oldest=std::min_element(weaponEffectsDiagnosticsV1_.begin(),weaponEffectsDiagnosticsV1_.end(),
                [](const auto& a,const auto& b){return a.second.retainedUntilS<b.second.retainedUntilS;});
            weaponEffectsDiagnosticsV1_.erase(oldest);
        }
        WeaponEffectsDiagnosticCacheV1 cache;cache.generation=generation;
        cache.name=std::string(name);cache.store=std::string(store);
        cache.value=std::make_unique<ipc::WeaponDiagnosticEffectsV1>();
        found=weaponEffectsDiagnosticsV1_.emplace(key,std::move(cache)).first;
        // Configuration can be captured only from this exact source lifetime.
        // A later occupant of the same ID is never consulted for old effects.
        if(const auto* source=entry(weapon);source&&source->info.generation==generation&&source->fuzeAdapter) {
            const auto& spec=source->fuzeAdapter->spec();auto& effects=*found->second.value;
            effects.flags|=ipc::WeaponDiagnosticFuzeV1;effects.fuzeAlgorithm=spec.fuzeAlgorithm;
            effects.armingTimeS=spec.armingS;effects.fuzeRadiusM=spec.fuzeM;
            if(burstEpoch-source->info.launchTimeS>=spec.armingS&&!source->origin.jettisoned)
                effects.flags|=ipc::WeaponDiagnosticArmedV1;
        }
    }
    auto& cache=found->second;auto& effects=*cache.value;
    // Maintained native weapons emit once per release. A manual diagnostic
    // that emits two independent bursts under one lifetime cannot alias their
    // particles or component records into a fabricated single source.
    if((effects.flags&ipc::WeaponDiagnosticBurstV1)&&effects.burstTimeS!=burstEpoch) {
        effects.stamp.status=ipc::WeaponDiagnosticStatusV1::Rejected;
        ipc::weaponDiagnosticTextV1(effects.stamp.cause,"Multiple burst epochs share this source generation; diagnostic source is ambiguous");
        ++cache.revision;cache.retainedUntilS=simTime_+30;return;
    }
    if(effects.stamp.status==ipc::WeaponDiagnosticStatusV1::Rejected||
       effects.stamp.status==ipc::WeaponDiagnosticStatusV1::CapacityExceeded)return;
    cache.retainedUntilS=simTime_+30;++cache.revision;
    effects.stamp.status=ipc::WeaponDiagnosticStatusV1::Available;effects.stamp.sourceId=weapon;
    effects.stamp.sampleTimeS=epoch;effects.stamp.processingTimeS=simTime_;effects.stamp.validUntilS=epoch;
    ipc::weaponDiagnosticTextV1(effects.stamp.cause,"Actual causal source field at captured native epoch; original generation and ordinals retained");
    effects.flags|=ipc::WeaponDiagnosticBurstV1|ipc::WeaponDiagnosticWarheadV1;
    effects.flags&=~static_cast<std::uint32_t>(ipc::WeaponDiagnosticConnectedRingV1);
    effects.burstTimeS=burstEpoch;effects.explosiveKg=explosiveKg;effects.warheadTopology=topology;
    effects.burstPositionEcefM[0]=field.burst.at.x;effects.burstPositionEcefM[1]=field.burst.at.y;effects.burstPositionEcefM[2]=field.burst.at.z;
    effects.caseMassKg=field.caseKg;
    effects.totalFragmentCount=static_cast<std::uint32_t>(std::min<std::size_t>(field.projectiles.size(),std::numeric_limits<std::uint32_t>::max()));
    effects.fragmentCount=static_cast<std::uint32_t>(std::min<std::size_t>(field.projectiles.size(),ipc::kWeaponDiagnosticFragmentsV1));
    for(std::size_t i=0;i<field.projectiles.size();++i) {
        const auto& native=field.projectiles[i];
        if(native.segment&&field.ageS<native.breakupAgeS)effects.flags|=ipc::WeaponDiagnosticConnectedRingV1;
        if(i>=effects.fragmentCount)continue;
        auto& row=effects.fragments[i];row={};row.id=native.sourceOrdinalV1;row.sampleTimeS=epoch;
        row.positionEcefM[0]=native.at.x;row.positionEcefM[1]=native.at.y;row.positionEcefM[2]=native.at.z;
        row.velocityEcefMs[0]=native.velocity.x;row.velocityEcefMs[1]=native.velocity.y;row.velocityEcefMs[2]=native.velocity.z;
        row.massKg=native.massKg;row.areaM2=native.areaM2;row.flags=native.segment?1u:0u;
    }
}
void World::captureWeaponImpactDiagnosticsV1(std::uint32_t weapon,std::uint64_t generation,
    const Entry& victim,const ms::damage::Impact& impact) {
    if(!options_.publish||!weapon||!generation)return;
    const auto found=weaponEffectsDiagnosticsV1_.find({weapon,generation});
    if(found==weaponEffectsDiagnosticsV1_.end())return;
    auto& cache=found->second;auto& effects=*cache.value;
    if(effects.stamp.status!=ipc::WeaponDiagnosticStatusV1::Available)return;
    for(const auto& component:impact.components) {
        if(component.component<0||component.hits<0||!std::isfinite(component.energyJ)||component.energyJ<0||
            (component.hits==0&&component.energyJ==0))continue;
        if(effects.totalDamageCount==std::numeric_limits<std::uint32_t>::max()) {
            effects.stamp.status=ipc::WeaponDiagnosticStatusV1::CapacityExceeded;
            ipc::weaponDiagnosticTextV1(effects.stamp.cause,"Native component-impact record count exceeds diagnostic integer capacity");
            ++cache.revision;return;
        }
        // Total counts observed component records, not physical punctures.
        // Actual distributed work is a record even when its hitCount is zero.
        ++effects.totalDamageCount;
        ++cache.revision;cache.retainedUntilS=simTime_+30;
        effects.stamp.processingTimeS=simTime_;
        ipc::WeaponDiagnosticDamageV1* row=nullptr;
        for(std::uint32_t i=0;i<effects.damageCount;++i) {
            auto& candidate=effects.damage[i];
            if(candidate.victim==victim.info.id&&candidate.victimGeneration==victim.info.generation&&
               candidate.component==static_cast<std::uint32_t>(component.component)){row=&candidate;break;}
        }
        if(!row) {
            if(effects.damageCount==ipc::kWeaponDiagnosticDamageV1)continue;
            row=&effects.damage[effects.damageCount++];*row={};row->victim=victim.info.id;row->victimGeneration=victim.info.generation;
            row->component=static_cast<std::uint32_t>(component.component);
            // The source airframe's kind is a string, not a wire enum. Keep
            // kind unknown rather than inventing a numeric equipment type.
        }
        const auto hits=static_cast<std::uint32_t>(component.hits);
        const double energy=row->penetratedEnergyJ+component.energyJ;
        if(hits>std::numeric_limits<std::uint32_t>::max()-row->hitCount||!std::isfinite(energy)) {
            effects.stamp.status=ipc::WeaponDiagnosticStatusV1::CapacityExceeded;
            ipc::weaponDiagnosticTextV1(effects.stamp.cause,"Native component impact accumulation exceeds diagnostic numeric capacity");
            return;
        }
        row->flags=ipc::WeaponDiagnosticImpactCountV1|ipc::WeaponDiagnosticImpactEnergyV1;row->sampleTimeS=simTime_;
        row->hitCount+=hits;row->penetratedEnergyJ=energy;
        // Native Impact does not supply continuous health or removed mass.
        // Those flags remain unavailable rather than turning default zero into death.
    }
}
void World::fillWeaponEffectsDiagnosticsV1(ipc::WeaponDiagnosticFrameV1& frame) {
    pruneWeaponEffectsDiagnosticsV1();
    const auto found=weaponEffectsDiagnosticsV1_.find({frame.entity,frame.generation});
    if(found==weaponEffectsDiagnosticsV1_.end())return;
    auto& cache=found->second;
    if(cache.publishedRevision!=cache.revision) {
        cache.value->stamp.sourceFrame=frame.worldFrame;cache.value->stamp.deliveredTimeS=frame.simTimeS;
        cache.publishedRevision=cache.revision;
    }
    const auto previous=frame.effects;frame.effects=*cache.value;
    if(!(frame.effects.flags&ipc::WeaponDiagnosticFuzeV1)&&(previous.flags&ipc::WeaponDiagnosticFuzeV1)) {
        frame.effects.flags|=previous.flags&(ipc::WeaponDiagnosticFuzeV1|ipc::WeaponDiagnosticArmedV1);
        frame.effects.fuzeAlgorithm=previous.fuzeAlgorithm;frame.effects.armingTimeS=previous.armingTimeS;frame.effects.fuzeRadiusM=previous.fuzeRadiusM;
    }
    if(!frame.name[0])ipc::weaponDiagnosticTextV1(frame.name,std::string_view(cache.name).substr(0,sizeof(frame.name)-1));
    if(!frame.store[0])ipc::weaponDiagnosticTextV1(frame.store,std::string_view(cache.store).substr(0,sizeof(frame.store)-1));
}
}
