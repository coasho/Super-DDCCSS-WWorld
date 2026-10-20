#include "simulation/runtime/world/World.h"
#include "contracts/world_stream/WeaponControlsLayout.h"
#include <cmath>
#include <set>

namespace fsim::session {
std::optional<ms::WeaponControlSnapshotV1> World::weaponControlsV1(std::uint32_t weapon) const {
    const auto* e=entry(weapon);
    if(!e || e->info.role!=ms::Role::Weapon || e->weaponEnded) return std::nullopt;
    ms::WeaponControlSnapshotV1 out;
    out.entity=weapon; out.slot=static_cast<std::uint32_t>(e->slot); out.generation=e->info.generation;
    out.sampleTimeS=simTime_; out.controllerAvailable=e->finControllerAvailable;
    out.controllerTimeS=e->finControllerTimeS;
    out.commands={e->inputs.aileron,e->inputs.elevator,e->inputs.rudder};
    auto& model=pool_->vehicle(e->slot);
    auto& cache=e->finTelemetry;
    const auto value=[](const PropertyHandle& h) {
        return h.valid()?h.get():std::numeric_limits<double>::quiet_NaN();
    };
    if(!cache.bound) {
        cache.bound=true;
        // Inventory queries are quiet for optional absent roots. Resolve only
        // known present leaves once; ordinary bombs/tanks never warn per frame.
        std::set<std::string> metadata,fcs;
        for(const auto& property:model.properties("fsim/missile-control")) metadata.insert(property.first);
        const auto bind=[&](const std::set<std::string>& names,const std::string& prefix,const std::string& leaf) {
            return names.count(leaf)?model.property(prefix+leaf):PropertyHandle{};
        };
        cache.version=bind(metadata,"fsim/missile-control/","version");
        cache.count=bind(metadata,"fsim/missile-control/","fin-count");
        const double count=value(cache.count);
        if(value(cache.version)==1 && std::isfinite(count) && count>=1 &&
           count<=ipc::kWeaponControlMaxFins && count==std::floor(count)) {
            for(const auto& property:model.properties("fcs")) fcs.insert(property.first);
            cache.fins.resize(static_cast<std::size_t>(count));
            for(std::size_t i=0;i<cache.fins.size();++i) {
                auto& row=cache.fins[i];
                const auto fin="fin-"+std::to_string(i);
                row[0]=bind(metadata,"fsim/missile-control/",fin+"-id");
                row[1]=bind(fcs,"fcs/",fin+"-delta-rad");
                row[2]=bind(fcs,"fcs/",fin+"-demand-rad");
                row[3]=bind(metadata,"fsim/missile-control/",fin+"-min-rad");
                row[4]=bind(metadata,"fsim/missile-control/",fin+"-max-rad");
                row[5]=bind(metadata,"fsim/missile-control/",fin+"-rate-rad_s");
                row[6]=bind(fcs,"fcs/",fin+"-actuator/malfunction/fail_stuck");
                row[7]=bind(fcs,"fcs/",fin+"-actuator/malfunction/fail_zero");
                row[8]=bind(fcs,"fcs/",fin+"-actuator/malfunction/fail_hardover");
            }
        }
    }
    const double count=value(cache.count);
    if(value(cache.version)!=1 || !std::isfinite(count) || count<1 ||
       count!=static_cast<double>(cache.fins.size())) return out;
    std::set<std::uint32_t> ids;
    for(std::uint32_t i=0;i<static_cast<std::uint32_t>(count);++i) {
        ms::WeaponFinStateV1 fin; fin.nativeIndex=i;
        const auto& row=cache.fins[i];
        const double id=value(row[0]);
        if(!std::isfinite(id) || id<1 || id>std::numeric_limits<std::uint32_t>::max() || id!=std::floor(id)) {
            out.fins.clear(); return out;
        }
        fin.id=static_cast<std::uint32_t>(id);
        if(!ids.insert(fin.id).second) { out.fins.clear(); return out; }
        fin.actualRad=value(row[1]); fin.demandRad=value(row[2]);
        fin.minimumRad=value(row[3]); fin.maximumRad=value(row[4]); fin.rateRadS=value(row[5]);
        const double stuck=value(row[6]),zero=value(row[7]),hard=value(row[8]);
        fin.valid=std::isfinite(fin.actualRad) && std::isfinite(fin.demandRad) &&
            std::isfinite(fin.minimumRad) && std::isfinite(fin.maximumRad) && fin.minimumRad<fin.maximumRad &&
            std::isfinite(fin.rateRadS) && fin.rateRadS>0 && std::isfinite(stuck) && std::isfinite(zero) && std::isfinite(hard);
        fin.failedStuck=stuck!=0 && std::isfinite(stuck); fin.failedZero=zero!=0 && std::isfinite(zero);
        fin.failedHardover=hard!=0 && std::isfinite(hard);
        out.fins.push_back(fin);
    }
    out.available=true;
    return out;
}

std::vector<ms::WeaponControlSnapshotV1> World::collectWeaponControls() const {
    std::vector<ms::WeaponControlSnapshotV1> out;
    for(const auto& e:entries_) if(e && e->info.role==ms::Role::Weapon)
        if(auto sample=weaponControlsV1(e->info.id)) out.push_back(std::move(*sample));
    return out;
}
}
