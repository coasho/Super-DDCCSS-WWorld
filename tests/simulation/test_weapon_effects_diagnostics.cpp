#include "mode_flights.h"
#include "foundation/transport/WeaponDiagnosticsV1.h"
#include "foundation/transport/WeaponDiagnosticCaptureBindingV1.h"
#include "foundation/transport/Recording.h"
#include "simulation/mission_systems/damage/Damage.h"
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <array>
#include <memory>
#include <set>
#include <fstream>

namespace {
using namespace fsim;
std::string diagnosticWorldName(){return "effects-diag-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());}
session::Detonation actualBurst(const sim::VehicleState& victim,std::uint32_t source){
    session::Detonation d;d.store="aim-120c";d.weapon=source;d.stepFraction=1;
    const auto* c=victim.rotationBodyToEcef;
    for(unsigned i=0;i<3;++i){
        d.positionEcef[i]=victim.positionEcef[i];
        d.velocityEcef[i]=c[3*i]*(victim.velocityBodyMs[0]+600)+c[3*i+1]*victim.velocityBodyMs[1]+c[3*i+2]*victim.velocityBodyMs[2];
        d.axisEcef[i]=d.velocityEcef[i];
    }
    return d;
}

TEST_CASE("Actual World closes and binds only its own flight and diagnostic capture", "[weapon-effects-diagnostics][world][weapon-diagnostic-binding]"){
    const auto prefix=std::filesystem::temp_directory_path()/diagnosticWorldName();
    const std::array<std::filesystem::path,2> paths={prefix.string()+"-a.fsrec",prefix.string()+"-b.fsrec"};
    struct Cleanup {
        std::array<std::filesystem::path,2> paths;
        ~Cleanup(){for(const auto& path:paths){std::error_code ignored;std::filesystem::remove(path,ignored);
            const auto diagnostics=std::filesystem::path(path.string()+".wdiag");std::filesystem::remove(diagnostics,ignored);
            std::filesystem::remove(ipc::weaponDiagnosticCaptureBindingPathV1(diagnostics),ignored);}}
    } cleanup{paths};
    std::array<std::uint32_t,2> source{};std::array<std::uint64_t,2> generation{};std::array<double,2> time{};
    for(std::size_t i=0;i<paths.size();++i){
        const auto name=diagnosticWorldName();auto options=modes::options(name.c_str());options.publish=true;
        options.recordPath=paths[i];options.recordIntervalSeconds=0;options.publishIntervalSeconds=0;
        {
            session::World world(options);const auto victim=modes::wing(world,"f16c",6000+1000*static_cast<double>(i),220);
            ms::WeaponLaunch release;release.type="jsbsim:aim120c";release.offsetBodyM[1]=30;
            source[i]=world.launchWeapon(victim,release);REQUIRE(source[i]);world.step(2);world.publishNow();
            generation[i]=world.info(source[i])->generation;time[i]=world.simTime();
            // A live/incomplete capture never gets a completion marker.
            CHECK_FALSE(std::filesystem::exists(ipc::weaponDiagnosticCaptureBindingPathV1(paths[i].string()+".wdiag")));
        }
        ipc::WeaponDiagnosticCaptureBindingV1 receipt;std::string error;
        REQUIRE(ipc::verifyWeaponDiagnosticCaptureBindingV1(paths[i],paths[i].string()+".wdiag",receipt,error));
        CHECK(receipt.publisherPid==platform::currentProcessId());CHECK(receipt.publisherIncarnation>0);
        ipc::Recording flight;REQUIRE(flight.load(paths[i]));REQUIRE_FALSE(flight.frames().empty());
        CHECK(flight.frames().back().simTime==time[i]);
        ipc::BoundWeaponDiagnosticsReplayV1 replay;REQUIRE(replay.open(paths[i],paths[i].string()+".wdiag",error));
        auto frame=std::make_unique<ipc::WeaponDiagnosticFrameV1>();REQUIRE(replay.atTime(time[i],*frame));
        CHECK(frame->entity==source[i]);CHECK(frame->generation==generation[i]);CHECK(frame->simTimeS==time[i]);
        CHECK(frame->worldFrame>0);CHECK(frame->selectionStatus==ipc::WeaponDiagnosticStatusV1::Available);
    }
    CHECK(source[0]==source[1]);CHECK(generation[0]==generation[1]);CHECK(time[0]==time[1]);
    ipc::BoundWeaponDiagnosticsReplayV1 mixed;std::string error;
    CHECK_FALSE(mixed.open(paths[0],paths[1].string()+".wdiag",error));CHECK_FALSE(error.empty());
    // A later capture without truth publication still invalidates an earlier
    // completion BEFORE truncation and keeps its legacy flight replay usable.
    {
        const auto name=diagnosticWorldName();auto options=modes::options(name.c_str());options.recordPath=paths[0];
        options.publish=false;session::World world(options);modes::wing(world,"f16c",6000,220);world.step(2);
        CHECK_FALSE(std::filesystem::exists(ipc::weaponDiagnosticCaptureBindingPathV1(paths[0].string()+".wdiag")));
    }
    ipc::Recording legacy;REQUIRE(legacy.load(paths[0]));CHECK_FALSE(legacy.frames().empty());
    CHECK_FALSE(mixed.open(paths[0],paths[0].string()+".wdiag",error));
}
std::unique_ptr<ipc::WeaponDiagnosticFrameV1> selectAndRead(session::World& world,ipc::WeaponDiagnosticsV1& reader,
                                                         const std::string& name,std::uint32_t source,std::uint64_t generation){
    world.publishNow();std::uint64_t pid=0,frame=0;std::uint32_t incarnation=0;
    REQUIRE(ipc::weaponDiagnosticWorldIdentityV1(name,pid,incarnation,frame));
    if(!reader.valid())REQUIRE(reader.open(name,pid,incarnation));
    REQUIRE(reader.requestSelection(source,generation,frame));world.publishNow();
    REQUIRE(ipc::weaponDiagnosticWorldIdentityV1(name,pid,incarnation,frame));
    auto out=std::make_unique<ipc::WeaponDiagnosticFrameV1>();REQUIRE(reader.read(frame,*out));
    REQUIRE(out->entity==source);REQUIRE(out->generation==generation);return out;
}
}

TEST_CASE("Actual emitted source ordinals survive causal flight and do not change absorption", "[weapon-effects-diagnostics][scene-effects]"){
    using namespace fsim::ms::damage;
    Warhead w;w.totalKg=.1;w.explosiveKg=.02;w.caseKg=.08;w.diameterM=.1;w.lengthM=.2;
    w.fragmentKg=.01;w.fragmentAreaM2=1e-5;w.fragments=8;
    w.caseThicknessM=.05-std::sqrt(.05*.05-w.caseKg/(7850*std::acos(-1.)*w.lengthM));
    w.speedMs=gurneyCylinder(w.gurneyMs,w.caseKg,w.explosiveKg);w.zones={{0,1,1}};
    Burst burst;burst.axis={1,0,0};fsim::Rng random(71);const auto field=emitFragmentFieldV1(w,burst,random);
    REQUIRE(field.valid);REQUIRE(field.projectiles.size()==8);
    for(std::size_t i=0;i<field.projectiles.size();++i)CHECK(field.projectiles[i].sourceOrdinalV1==i+1);
    SceneEnvironmentV1 environment;environment.airDensity=0;environment.maxFlightTimeS=.005;
    const auto flown=assessSceneV1(field,{},environment);REQUIRE(flown.valid);REQUIRE(flown.continuation.projectiles.size()==8);
    for(std::size_t i=0;i<field.projectiles.size();++i){
        CHECK(flown.continuation.projectiles[i].sourceOrdinalV1==field.projectiles[i].sourceOrdinalV1);
        CHECK(flown.continuation.projectiles[i].at.x>field.projectiles[i].at.x);
    }
    Airframe solid;Airframe::Component component;component.centre={20,0,0};component.half={.2,10,10};solid.components.push_back(component);
    Target target;target.airDensity=0;target.bodyToEcef={V3{1,0,0},V3{0,1,0},V3{0,0,1}};
    auto unlabelled=flown.continuation;for(auto& p:unlabelled.projectiles)p.sourceOrdinalV1=0;
    environment.maxFlightTimeS=.03;
    const std::vector<SceneVictimV1> victims={{1,AttachedGeometryView(solid),target}};
    const auto labelledImpact=assessSceneV1(flown.continuation,victims,environment);
    const auto unlabelledImpact=assessSceneV1(unlabelled,victims,environment);
    REQUIRE(labelledImpact.valid);REQUIRE(unlabelledImpact.valid);REQUIRE(labelledImpact.victims.size()==1);
    REQUIRE(labelledImpact.victims[0].impact.components.size()==1);CHECK(labelledImpact.continuation.projectiles.empty());
    CHECK(labelledImpact.absorbedMassKg==unlabelledImpact.absorbedMassKg);
    CHECK(labelledImpact.transferredMomentumEcef.x==unlabelledImpact.transferredMomentumEcef.x);
    CHECK(labelledImpact.transferredMomentumEcef.y==unlabelledImpact.transferredMomentumEcef.y);
    CHECK(labelledImpact.transferredMomentumEcef.z==unlabelledImpact.transferredMomentumEcef.z);
    CHECK(labelledImpact.victims[0].impact.components[0].energyJ==unlabelledImpact.victims[0].impact.components[0].energyJ);
}

TEST_CASE("Native effect diagnostics retain distinct source generations through reset and removal", "[weapon-effects-diagnostics][world][causal-effects]"){
    const auto name=diagnosticWorldName();auto options=fsim::modes::options(name.c_str());options.publish=true;
    session::World world(options);const auto victim=fsim::modes::wing(world,"f16c",6000,220);
    ms::WeaponLaunch release;release.type="jsbsim:aim120c";release.offsetBodyM[1]=30;
    const auto source=world.launchWeapon(victim,release);REQUIRE(source);world.step(2);
    const auto firstGeneration=world.info(source)->generation;ipc::WeaponDiagnosticsV1 reader;
    auto out=selectAndRead(world,reader,name,source,firstGeneration);
    const double firstBurst=world.simTime();world.detonateCapturedV1(actualBurst(*world.vehicleState(victim),source));
    out=selectAndRead(world,reader,name,source,firstGeneration);
    REQUIRE(out->effects.flags&ipc::WeaponDiagnosticBurstV1);REQUIRE(out->effects.fragmentCount>0);
    CHECK(out->effects.burstTimeS==firstBurst);std::set<std::uint64_t> originalOrdinals;
    for(std::uint32_t i=0;i<out->effects.fragmentCount;++i){CHECK(out->effects.fragments[i].id>0);originalOrdinals.insert(out->effects.fragments[i].id);}
    CHECK(originalOrdinals.size()==out->effects.fragmentCount);
    auto initial=world.info(victim)->initial;initial.longitudeDeg+=.1;
    REQUIRE(world.resetVehicle(source,&initial));const auto secondGeneration=world.info(source)->generation;
    REQUIRE(secondGeneration>firstGeneration);
    out=selectAndRead(world,reader,name,source,secondGeneration);CHECK_FALSE(out->effects.flags&ipc::WeaponDiagnosticBurstV1);
    world.step();const double secondBurst=world.simTime();
    world.detonateCapturedV1(actualBurst(*world.vehicleState(victim),source));
    out=selectAndRead(world,reader,name,source,secondGeneration);
    REQUIRE(out->effects.flags&ipc::WeaponDiagnosticBurstV1);CHECK(out->effects.burstTimeS==secondBurst);
    REQUIRE(world.removeVehicle(source));world.step();
    out=selectAndRead(world,reader,name,source,firstGeneration);
    REQUIRE(out->effects.flags&ipc::WeaponDiagnosticBurstV1);CHECK(out->effects.burstTimeS==firstBurst);
    CHECK(out->selectionStatus==ipc::WeaponDiagnosticStatusV1::Removed);
    REQUIRE(out->eventCount>0);
    for(std::uint32_t i=0;i<out->eventCount;++i) {
        const auto& event=out->events[i];
        if(event.source==source){REQUIRE(event.flags&ipc::WeaponDiagnosticEventSourceGenerationV1);CHECK(event.sourceGeneration==firstGeneration);}
        if(event.target==source){REQUIRE(event.flags&ipc::WeaponDiagnosticEventTargetGenerationV1);CHECK(event.targetGeneration==firstGeneration);}
    }
    CHECK(out->effects.stamp.sampleTimeS<=world.simTime());
    for(std::uint32_t i=0;i<out->effects.fragmentCount;++i){CHECK(out->effects.fragments[i].id>0);CHECK(out->effects.fragments[i].sampleTimeS==out->effects.stamp.sampleTimeS);}
    for(std::uint32_t i=0;i<out->effects.damageCount;++i){const auto& row=out->effects.damage[i];CHECK_FALSE(row.flags&ipc::WeaponDiagnosticComponentHealthV1);CHECK_FALSE(row.flags&ipc::WeaponDiagnosticRemovedMassV1);}
    const double delivered=out->effects.stamp.deliveredTimeS;world.publishNow();
    out=selectAndRead(world,reader,name,source,firstGeneration);CHECK(out->effects.stamp.deliveredTimeS==delivered);
    out=selectAndRead(world,reader,name,source,secondGeneration);
    REQUIRE(out->effects.flags&ipc::WeaponDiagnosticBurstV1);CHECK(out->effects.burstTimeS==secondBurst);
}
