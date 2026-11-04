// Existing external-tank fuel functions only. Diagnostic mounts and referee
// inspection never assert OEM integration, fragment vulnerability or M10 tasks.
#include "mode_flights.h"
#include "simulation/flight/JsbsimModel.h"
#include "foundation/core/Json.h"
#include "foundation/transport/WeaponDiagnosticCaptureBindingV1.h"
#include <FGFDMExec.h>
#include <models/FGPropulsion.h>
#include <models/propulsion/FGTank.h>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace {
using namespace fsim;
using core::Json;
namespace fs=std::filesystem;
constexpr double kgPerLb=.45359237;
const fs::path source=FSIM_TEST_SOURCE_DIR;
using V=std::array<double,3>;
V add(V a,const V& b,double scale=1){for(unsigned k=0;k<3;++k)a[k]+=scale*b[k];return a;}
V times(V a,double value){for(auto& v:a)v*=value;return a;}
double norm(const V& a){return std::hypot(a[0],a[1],a[2]);}
Json vector(const V& a){return Json::Array{a[0],a[1],a[2]};}
void close(double a,double b,double budget){REQUIRE(std::isfinite(a));REQUIRE(std::isfinite(b));CHECK(std::abs(a-b)<=budget);}
std::string hash(const fs::path& path){ipc::WeaponDiagnosticFileDigestV1 out;std::string error;
    REQUIRE(ipc::weaponDiagnosticFileDigestV1(path,out,error));return out.sha256;}
Json file(const fs::path& path){return Json::Object{{"path",fs::absolute(path).generic_string()},{"sha256",hash(path)}};}
Json runtime(){
    Json out=Json::Object{{"kind","runtime"},{"schema","fsim.external-tank-functions.v1"},{"seed","17"}};
#ifdef _WIN32
    for(const auto& item:std::initializer_list<std::pair<const char*,const wchar_t*>>{{"binary",nullptr},{"jsbsim",L"libJSBSim.dll"}}){
        const auto module=GetModuleHandleW(item.second);REQUIRE(module);wchar_t path[32768]{};
        const auto count=GetModuleFileNameW(module,path,32768);REQUIRE(count>0);REQUIRE(count<32768);
        out.set(item.first,file(fs::path(path)));
    }
#else
    FAIL("Maintained native population provenance requires Windows presets");
#endif
    return out;
}
struct Mount {std::string aircraft,payload,basis;int station=0;fs::path fit;};
std::map<std::string,Mount> mounts(){
    std::map<std::string,Mount> result;std::vector<fs::path> fits;
    for(const auto& entry:fs::directory_iterator(source/"aircraft")){
        if(entry.path().filename()==".codex"||entry.is_symlink()||!entry.is_directory())continue;
        if(fs::is_regular_file(entry.path()/"payloads.json"))fits.push_back(entry.path()/"payloads.json");
    }
    std::sort(fits.begin(),fits.end());
    for(const auto& path:fits){std::vector<std::string> errors;const auto fit=ms::readFit(path,source/"assets/payloads",errors);
        REQUIRE(fit);REQUIRE(errors.empty());
        for(const auto& load:fit->loadouts)for(const auto& item:load.items){
            const auto card=fit->stores.find(item.store);REQUIRE(card!=fit->stores.end());
            if(card->second->storeClass=="tank")result.try_emplace(item.store,Mount{fit->aircraft,item.payload,"authored_loadout",item.station,path});
        }
    }
    return result;
}
double property(sim::FlightModel& model,const std::string& name){const auto p=model.property(name);REQUIRE(p.valid());
    const double value=p.get();REQUIRE(std::isfinite(value));return value;}
std::string indexed(const std::string& base,unsigned index){return base+(index?"["+std::to_string(index)+"]":"");}
V location(sim::FlightModel& model,unsigned index){V result{};unsigned k=0;
    for(const auto* axis:{"X","Y","Z"})result[k++]=property(model,indexed(std::string("inertia/pointmass-location-")+axis+"-inches",index))*.0254;
    return result;
}
std::vector<double> weights(sim::FlightModel& model){std::vector<double> result;
    for(unsigned i=0;i<1024;++i){const auto p=model.property(indexed("inertia/pointmass-weight-lbs",i));if(!p.valid())break;
        result.push_back(p.get()*kgPerLb);}REQUIRE(!result.empty());return result;
}
struct Mass {double kg=0,fuel=0;V cg{},fuelMoment{};};
Mass mass(sim::JsbsimModel& model){const auto native=model.rigid();Mass out;out.kg=native.massKg;
    std::copy(std::begin(native.cgStructural),std::end(native.cgStructural),out.cg.begin());
    const auto propulsion=model.exec().GetPropulsion();REQUIRE(propulsion);REQUIRE(propulsion->GetNumTanks()<=128);
    for(unsigned i=0;i<propulsion->GetNumTanks();++i){const auto tank=propulsion->GetTank(i);REQUIRE(tank);
        const double kg=tank->GetContents()*kgPerLb;REQUIRE(std::isfinite(kg));REQUIRE(kg>=0);out.fuel+=kg;
        // GetXYZ includes the native contents-dependent drain-location law.
        // Tied x/y/z-position properties describe the nominal full location.
        V at{};for(int axis=1;axis<=3;++axis)at[static_cast<std::size_t>(axis-1)]=tank->GetXYZ(axis)*.0254;
        out.fuelMoment=add(out.fuelMoment,at,kg);
    }
    REQUIRE(out.kg>0);return out;
}
Json massJson(const Mass& value){return Json::Object{{"mass_kg",value.kg},{"cg_structural_m",vector(value.cg)},
    {"internal_fuel_kg",value.fuel},{"internal_fuel_moment_kg_m",vector(value.fuelMoment)}};}
struct Run {Json json;double fuel=0,kg=0;V cg{},fuelAt{};};
Run run(const ms::StoreCard& card,const Mount& mount,bool empty){
    INFO(card.id<<" empty="<<empty<<" mount="<<mount.aircraft<<":"<<mount.station);
    auto options=modes::options(("external-tank-"+card.id+(empty?"-empty":"-wet")).c_str());options.frameSkip=1;
    session::World world(options);session::VehicleSpec initial;initial.name=mount.aircraft+"-external-dive";
    initial.type="jsbsim:"+mount.aircraft;initial.initial.altitudeMslM=120;initial.initial.airspeedTrueMs=180;
    initial.initial.headingDeg=90;initial.initial.pitchDeg=-60;initial.initial.fuelFraction=.1;
    const auto carrier=world.createVehicle(initial);REQUIRE(carrier);world.step();
    auto* native=dynamic_cast<sim::JsbsimModel*>(world.model(carrier));REQUIRE(native);auto* host=world.missionSystems(carrier);REQUIRE(host);
    std::uint16_t payload=0;bool found=false;
    for(const auto& p:host->payloads())if(p.family==ms::Family::StoreStation&&(mount.payload.empty()||p.name==mount.payload)){
        const auto* stationCard=host->card(p.index);if(stationCard&&stationCard->station(mount.station)){payload=p.index;found=true;break;}}
    REQUIRE(found);const auto* station=host->card(payload)->station(mount.station);REQUIRE(station);
    REQUIRE(std::find(station->takes.begin(),station->takes.end(),"tank")!=station->takes.end());
    REQUIRE(station->maxKg>=card.value("mass_kg",0)+card.value("fuel_kg",0));
    const auto initialWeights=weights(*native);
    REQUIRE(world.loadStore(carrier,payload,mount.station,card.id)==ms::Reason::None);world.step();
    REQUIRE(world.stores(carrier).size()==1);const double capacity=card.value("fuel_kg",0);REQUIRE(capacity>0);
    REQUIRE(native->energy().fuelCapacityKg-native->energy().fuelKg>host->externalFuelKg());
    const double externalBefore=host->externalFuelKg(),internalBefore=native->energy().fuelKg;
    // Isolate the existing component law from engine consumption at one fixed
    // boundary. Empty is produced by actual bounded transfer, never deletion.
    const double requestedTransferDuration=empty?capacity/10+1:1;
    const auto transferCalls=static_cast<unsigned>(std::ceil(requestedTransferDuration/world.dt()));
    REQUIRE(transferCalls>0);REQUIRE(transferCalls<=100000);
    // Use the SDK-owned seam through its World. The borrowed Host remains
    // const; these nominal transfer intervals do not advance the World clock.
    for(unsigned call=0;call<transferCalls;++call)world.transferFuel();
    const double transferDuration=static_cast<double>(transferCalls)*world.dt();
    const double externalAfter=host->externalFuelKg(),internalAfter=native->energy().fuelKg;
    const double transferred=externalBefore-externalAfter;
    REQUIRE(transferred>0);close(internalAfter-internalBefore,transferred,1e-7*std::max(1.,capacity));
    if(empty)close(externalAfter,0,1e-8);else REQUIRE(externalAfter>0);
    world.step();const auto loadedWeights=weights(*native);REQUIRE(loadedWeights.size()==initialWeights.size());
    std::vector<unsigned> changed;for(unsigned i=0;i<loadedWeights.size();++i)
        if(loadedWeights[i]-initialWeights[i]>.5*card.value("mass_kg",0))changed.push_back(i);
    REQUIRE(changed.size()==1);const auto stationAt=location(*native,changed[0]);
    const auto before=mass(*native);const double storeBefore=host->stationMassKg(payload,mount.station);
    const double remainingBefore=host->externalFuelKg();
    ms::JettisonCommand jettison;jettison.payload=payload;jettison.emergency=true;
    REQUIRE(world.submit(carrier,jettison).accepted());world.step();
    const auto stores=world.stores(carrier);REQUIRE(stores.size()==1);REQUIRE(stores[0].status==ms::StoreStatus::Jettisoned);
    const auto weapon=stores[0].weapon;REQUIRE(weapon);REQUIRE(world.weapons().size()==1);REQUIRE(world.weapons()[0].jettisoned);
    const auto after=mass(*native);auto* tank=dynamic_cast<sim::JsbsimModel*>(world.model(weapon));REQUIRE(tank);
    const double actualFuel=property(*tank,"inertia/pointmass-weight-lbs")*kgPerLb;
    close(actualFuel,stores[0].fuelKg,1e-7*std::max(1.,capacity));
    REQUIRE(actualFuel<=remainingBefore+1e-8);if(empty)close(actualFuel,0,1e-8);else REQUIRE(actualFuel>0);
    const double massError=(before.kg-after.kg)-(before.fuel-after.fuel)-storeBefore;
    const auto removedMoment=add(add(times(before.cg,before.kg),times(after.cg,after.kg),-1),add(before.fuelMoment,after.fuelMoment,-1),-1);
    const double momentError=norm(add(removedMoment,stationAt,-storeBefore));
    // Numerical budgets are fixed from independently read native mass/moment
    // magnitudes; they are not fitted to post-release discrepancies.
    const double massBudget=1e-6*std::max({1.,before.kg,after.kg});
    const double momentBudget=2e-6*std::max({1.,before.kg*norm(before.cg),after.kg*norm(after.cg)});
    close(massError,0,massBudget);CHECK(momentError<=momentBudget);
    const auto tankMass=mass(*tank);const auto fuelAt=location(*tank,0);
    const auto* releasedState=world.vehicleState(weapon);REQUIRE(releasedState);
    const double releaseDown=releasedState->velocityNedMs[2];REQUIRE(releaseDown>100);
    const auto xml=fs::path(tank->exec().GetFullAircraftPath().str())/(tank->exec().GetModelName()+".xml");REQUIRE(fs::is_regular_file(xml));
    const auto xmlHash=hash(xml);const auto slot=world.vehicleStreamSlot(weapon);REQUIRE(slot);
    unsigned impacts=0,fires=0,detonations=0;double fireKg=0,impactEnergy=0,continuing=0,impactTime=-1;
    for(unsigned step=0;step<modes::stepsFor(world,20)&&impacts==0;++step){world.step();
        for(const auto& event:world.takeEngagementEvents())if(event.slot==*slot){
            const auto kind=static_cast<ipc::EngagementKind>(event.kind);
            if(kind==ipc::EngagementKind::Impact){++impacts;impactEnergy=event.energyJ;continuing=event.rateKgS;impactTime=event.simTime;}
            if(kind==ipc::EngagementKind::Fire&&event.amountKg>0){++fires;fireKg+=event.amountKg;}
            if(kind==ipc::EngagementKind::Detonation)++detonations;
        }
    }
    if(impacts!=1){const auto* final=world.vehicleState(weapon);int terminal=-1;CAPTURE(weapon,world.simTime());
        if(final){CAPTURE(final->altitudeAglM,final->airspeedTrueMs,final->velocityNedMs[2]);}
        for(const auto& record:world.weapons())if(record.id==weapon)terminal=static_cast<int>(record.end);
        const double finalHeight=final?final->altitudeAglM:-1,finalSpeed=final?final->airspeedTrueMs:-1,finalDown=final?final->velocityNedMs[2]:-1;
        CAPTURE(terminal,finalHeight,finalSpeed,finalDown);REQUIRE(impacts==1);}
    REQUIRE(impacts==1);CHECK(impactEnergy>0);CHECK(continuing==0);CHECK(detonations==0);
    if(empty){CHECK(fires==0);CHECK(fireKg==0);}else{CHECK(fires==1);CHECK(fireKg>0);CHECK(fireKg<=actualFuel*(1+1e-12));}
    REQUIRE(hash(xml)==xmlHash);
    Run result;result.fuel=actualFuel;result.kg=tankMass.kg;result.cg=tankMass.cg;result.fuelAt=fuelAt;
    result.json=Json::Object{{"empty_control",empty},{"carrier",mount.aircraft},{"station",mount.station},{"fit_basis",mount.basis},
        {"initial_altitude_m",120},{"initial_airspeed_ms",180},{"initial_pitch_deg",-60},{"release_down_ms",releaseDown},
        {"carrier_xml",file(fs::path(native->exec().GetFullAircraftPath().str())/(native->exec().GetModelName()+".xml"))},
        {"loaded_xml",Json::Object{{"path",fs::absolute(xml).generic_string()},{"sha256",xmlHash}}},
        {"external_before_transfer_kg",externalBefore},{"external_after_transfer_kg",externalAfter},
        {"internal_before_transfer_kg",internalBefore},{"internal_after_transfer_kg",internalAfter},{"transfer_duration_s",transferDuration},
        {"transfer_calls",static_cast<int>(transferCalls)},{"native_dt_s",world.dt()},
        {"internal_capacity_kg",native->energy().fuelCapacityKg},
        {"before_release",massJson(before)},{"after_release",massJson(after)},{"store_before_release_kg",storeBefore},
        {"external_before_release_kg",remainingBefore},
        {"station_structural_m",vector(stationAt)},{"release_mass_error_kg",massError},{"release_mass_budget_kg",massBudget},
        {"release_moment_error_kg_m",momentError},{"release_moment_budget_kg_m",momentBudget},
        {"released_fuel_kg",actualFuel},{"released_mass_kg",tankMass.kg},{"released_cg_structural_m",vector(tankMass.cg)},
        {"fuel_point_structural_m",vector(fuelAt)},{"impact_count",static_cast<int>(impacts)},{"fire_count",static_cast<int>(fires)},
        {"fire_kg",fireKg},{"continuing_kg_s",continuing},{"detonation_count",static_cast<int>(detonations)},
        {"impact_energy_j",impactEnergy},{"impact_time_s",impactTime},{"frame_skip",1}};
    return result;
}
}

TEST_CASE("All authored external tanks transfer remaining fuel and preserve native release mass CG and ground fire", "[external-tank-population][stores][world]"){
    std::vector<std::string> errors;const auto library=ms::readStoreLibrary(source/"assets/stores",errors);REQUIRE(errors.empty());
    std::vector<std::shared_ptr<const ms::StoreCard>> tanks;for(const auto& [id,card]:library)if(card->storeClass=="tank")tanks.push_back(card);
    REQUIRE(tanks.size()==19);const auto authored=mounts();std::ofstream output;
    if(const auto* directory=std::getenv("FSIM_EXTERNAL_TANK_POPULATION_DIR")){
        const auto path=fs::path(directory)/"external-tanks.ndjson";fs::create_directories(path.parent_path());
        output.open(path,std::ios::binary|std::ios::trunc);REQUIRE(output.good());output<<runtime().dump()<<'\n';output.flush();REQUIRE(output.good());
    }
    for(const auto& card:tanks){INFO(card->id);const auto cardPath=source/"assets/stores"/(card->id+".json");const auto cardHash=hash(cardPath);
        const auto it=authored.find(card->id);Mount mount;
        if(it!=authored.end())mount=it->second;
        else {REQUIRE(card->id=="ptb-1150");mount={"f16c","","generic_diagnostic_class_mass_mount_not_OEM",4,source/"aircraft/f16c/payloads.json"};}
        const auto wet=run(*card,mount,false),empty=run(*card,mount,true);
        const double budget=1e-7*std::max(1.,wet.kg);close(wet.kg-empty.kg,wet.fuel,budget);
        const double momentBudget=1e-7*std::max({1.,wet.kg*norm(wet.cg),empty.kg*norm(empty.cg)});
        const double error=norm(add(add(times(wet.cg,wet.kg),times(empty.cg,empty.kg),-1),wet.fuelAt,-wet.fuel));CHECK(error<=momentBudget);
        REQUIRE(hash(cardPath)==cardHash);
        if(output.is_open()){Json row=Json::Object{{"kind","tank"},{"card",card->id},{"model",card->model},
            {"card_source",Json::Object{{"path",cardPath.generic_string()},{"sha256",cardHash}}},{"fit_source",file(mount.fit)},
            {"capacity_kg",card->value("fuel_kg",0)},{"dry_card_mass_kg",card->value("mass_kg",0)},
            {"wet",wet.json},{"empty",empty.json},{"wet_empty_mass_error_kg",wet.kg-empty.kg-wet.fuel},
            {"wet_empty_mass_budget_kg",budget},{"wet_empty_moment_error_kg_m",error},{"wet_empty_moment_budget_kg_m",momentBudget}};
            output<<row.dump()<<'\n';output.flush();REQUIRE(output.good());}
    }
    if(output.is_open()){output<<Json(Json::Object{{"kind","complete"},{"cards",19}}).dump()<<'\n';output.flush();REQUIRE(output.good());}
}
