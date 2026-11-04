// Exact authored-card effect execution. Synthetic vacuum absorption is a method
// oracle; actual native airframe damage and cruise terrain contact are separate
// integration observations. None establishes measured equipment lethality.
#include "simulation/runtime/world/World.h"
#include "simulation/mission_systems/components/registration/BuiltInProvidersV1.h"
#include "simulation/mission_systems/damage/State.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace {
using namespace fsim;
namespace dmg=ms::damage;
namespace cmp=ms::components;
std::ofstream receipt(const char* name,const char* header) {
    std::ofstream out;
    if(const auto* root=std::getenv("FSIM_EFFECTS_POPULATION_DIR")) {
        const auto path=std::filesystem::path(root)/name;
        std::filesystem::create_directories(path.parent_path());
        // Each executed case replaces its complete raw population from byte
        // zero. An interrupted case leaves a partial file, rejected by joins.
        out.open(path,std::ios::binary|std::ios::trunc);REQUIRE(out.good());out<<std::setprecision(17)<<header<<'\n';
    }
    return out;
}
void runtimeReceipt() {
    auto out=receipt("runtime.csv","role,path");if(!out.is_open())return;
#ifdef _WIN32
    for(const auto& [role,name]:std::initializer_list<std::pair<const char*,const wchar_t*>>{
            {"binary",nullptr},{"jsbsim",L"libJSBSim.dll"}}) {
        const auto module=GetModuleHandleW(name);REQUIRE(module);
        wchar_t path[32768]{};const DWORD size=GetModuleFileNameW(module,path,32768);
        REQUIRE(size>0);REQUIRE(size<32768);
        out<<role<<','<<std::quoted(std::filesystem::path(path).generic_string())<<'\n';
    }
#else
    FAIL("Raw effects provenance currently requires the maintained Windows native presets");
#endif
    out.flush();REQUIRE(out.good());
}
bool finite(dmg::V3 value) {return std::isfinite(value.x)&&std::isfinite(value.y)&&std::isfinite(value.z);}
dmg::V3 rotate(const sim::VehicleState& s,dmg::V3 v) {
    const auto* r=s.rotationBodyToEcef;
    return {r[0]*v.x+r[1]*v.y+r[2]*v.z,r[3]*v.x+r[4]*v.y+r[5]*v.z,r[6]*v.x+r[7]*v.y+r[8]*v.z};
}
session::WorldOptions options(int skip=4) {
    session::WorldOptions result;result.name="exact-authored-effects";result.publish=false;
    result.seed=17;result.workers=1;result.pinWorkers=false;result.frameSkip=skip;
    result.jsbsimRoot=FSIM_TEST_JSBSIM_ROOT;return result;
}
void sourceReceipt(const std::string& name,const dmg::FragmentFieldV1& field) {
    auto out=receipt(name.c_str(),"ordinal,mass_kg,area_m2,cd,segment,x,y,z,vx,vy,vz,ex,ey,ez,evx,evy,evz,expansion_per_s,material_length_m");
    if(!out.is_open())return;
    for(const auto& p:field.projectiles)out<<p.sourceOrdinalV1<<','<<p.massKg<<','<<p.areaM2<<','<<p.cd<<','<<p.segment<<','
        <<p.at.x<<','<<p.at.y<<','<<p.at.z<<','<<p.velocity.x<<','<<p.velocity.y<<','<<p.velocity.z<<','
        <<p.end.x<<','<<p.end.y<<','<<p.end.z<<','<<p.endVelocity.x<<','<<p.endVelocity.y<<','<<p.endVelocity.z<<','
        <<p.expansionPerS<<','<<p.materialLengthM<<'\n';
    out.flush();REQUIRE(out.good());
}
struct Probe {unsigned calls=0;std::size_t emitted=0;double mass=0;std::string card;};
class ObservedScene final:public dmg::WarheadAdapter,public dmg::WarheadSceneExecutionV2 {
public:
    ObservedScene(std::unique_ptr<dmg::WarheadAdapter> inner,std::shared_ptr<Probe> probe)
        :inner_(std::move(inner)),probe_(std::move(probe)){}
    std::string_view implementationId() const noexcept override {return inner_->implementationId();}
    const dmg::Warhead& model() const noexcept override {return inner_->model();}
    dmg::Impact assess(const dmg::Burst& b,const dmg::AttachedGeometryView& a,const dmg::Target& t,Rng& r) const override {
        return inner_->assess(b,a,t,r);
    }
    dmg::FragmentFieldV1 emitScene(const dmg::Burst& burst,Rng& random) const override {
        const auto* scene=dynamic_cast<const dmg::WarheadSceneExecutionV2*>(inner_.get());REQUIRE(scene);
        auto field=scene->emitScene(burst,random);++probe_->calls;probe_->emitted+=field.projectiles.size();
        for(const auto& p:field.projectiles)probe_->mass+=p.massKg;
        if(!probe_->card.empty())sourceReceipt("native-"+probe_->card+".csv",field);
        return field; // Observation only: no source or execution semantics change.
    }
private:
    std::unique_ptr<dmg::WarheadAdapter> inner_;std::shared_ptr<Probe> probe_;
};
cmp::ProviderRegistryV1 observedRegistry(const cmp::DescriptorV1& descriptor,std::shared_ptr<Probe> probe) {
    auto registry=cmp::builtInProvidersV1();const auto original=registry.warhead;std::string error;
    REQUIRE(registry.warhead.registerFactory(descriptor.implementationId,std::string(equipment::mechanism(descriptor.implementationId)),
        [original,probe](const auto& d,const auto& context,auto& why)->std::unique_ptr<dmg::WarheadAdapter> {
            auto inner=original.create(d,context,why);if(!inner)return {};
            return std::make_unique<ObservedScene>(std::move(inner),probe);
        },true,error));
    return registry;
}
bool finiteWorld(const session::World& world) {
    for(const auto id:world.vehicleIds()) {
        const auto* s=world.vehicleState(id);if(!s||s->diverged)return false;
        for(const double x:s->positionEcef)if(!std::isfinite(x))return false;
        for(const double x:s->velocityBodyMs)if(!std::isfinite(x))return false;
        for(const double x:s->angularRateBodyRadS)if(!std::isfinite(x))return false;
    }
    return true;
}
ms::WeaponRecord weapon(const session::World& world,std::uint32_t id) {
    for(const auto& row:world.weapons())if(row.id==id)return row;
    FAIL("Released native weapon history disappeared");return {};
}
}

TEST_CASE("Every authored explosive executes its selected finite scene and native component damage", "[effects-population][scene-effects]") {
    runtimeReceipt();
    auto out=receipt("population.csv","card,selection,mechanism,version,segments,connected,emitted,case_kg,emitted_kg,mass_tolerance_kg,reference_energy_j,absorbed_energy_j,energy_tolerance_j,momentum_error_ns,momentum_tolerance_ns,absorbed_kg,native_scene_calls,native_emitted,native_emitted_kg,native_fragment_hits,native_fuel_hole_m2,native_finite,blast_tnt_kg");
    std::vector<std::string> errors;const std::filesystem::path library=FSIM_TEST_STORE_DIR;
    const auto cards=ms::readStoreLibrary(library,errors);REQUIRE(errors.empty());REQUIRE(cards.size()==53);
    const auto registry=cmp::builtInProvidersV1();unsigned tested=0;
    for(const auto& [name,card]:cards) {
        cmp::ProfileV1 profile;std::string error;
        REQUIRE(cmp::readProfileV1(library/(name+".json"),*card,profile,error));
        const auto* descriptor=profile.find("warhead");REQUIRE(descriptor);
        if(descriptor->availability==cmp::AvailabilityV1::NotApplicable)continue;
        ++tested;INFO(name);REQUIRE(descriptor->availability==cmp::AvailabilityV1::Available);
        cmp::ProviderContextV1 context{*card,profile,{},std::nullopt};
        const auto adapter=registry.warhead.create(*descriptor,context,error);INFO(error);REQUIRE(adapter);
        const auto* scene=dynamic_cast<const dmg::WarheadSceneExecutionV2*>(adapter.get());REQUIRE(scene);
        const auto& model=adapter->model();const auto mechanism=equipment::mechanism(descriptor->implementationId);
        REQUIRE(adapter->implementationId()==mechanism);
        dmg::Burst burst;burst.axis={1,0,0};burst.velocity={100,20,-30};burst.angularVelocityEcef={2,-3,4};
        Rng random(93);const auto field=scene->emitScene(burst,random);REQUIRE(field.valid);
        sourceReceipt("method-"+name+".csv",field);
        REQUIRE(field.projectiles.size()==static_cast<std::size_t>(model.fragments));REQUIRE_FALSE(field.projectiles.empty());
        double mass=0,referenceEnergy=0,momentumScale=0,extent=1;dmg::V3 referenceMomentum;unsigned segments=0,connected=0;
        const dmg::V3 targetVelocity{17,-8,3};
        for(const auto& p:field.projectiles) {
            REQUIRE(finite(p.at));REQUIRE(finite(p.velocity));REQUIRE(p.massKg>0);REQUIRE(std::isfinite(p.areaM2));REQUIRE(p.areaM2>0);
            mass+=p.massKg;const auto mean=p.segment?(p.velocity+p.endVelocity)*.5:p.velocity;
            const auto relative=mean-targetVelocity;
            // A uniform line's kinetic energy is COM translation plus the
            // integral of its linear velocity gradient: m |v1-v0|^2 / 24.
            referenceEnergy+=.5*p.massKg*dmg::dot(relative,relative);
            referenceMomentum+=relative*p.massKg;momentumScale+=p.massKg*dmg::length(relative);
            extent=std::max(extent,dmg::length(p.at));
            if(p.segment) {
                ++segments;REQUIRE(finite(p.end));REQUIRE(finite(p.endVelocity));
                const auto difference=p.endVelocity-p.velocity;
                referenceEnergy+=p.massKg*dmg::dot(difference,difference)/24;
                extent=std::max(extent,dmg::length(p.end));
            }
            if(p.expansionPerS>0)++connected;
        }
        const double budget=512*std::numeric_limits<double>::epsilon()*static_cast<double>(field.projectiles.size());
        const double massTolerance=budget*std::max(1.,model.caseKg),energyTolerance=budget*std::max(1.,referenceEnergy);
        const double momentumTolerance=budget*std::max(1.,momentumScale);
        REQUIRE(std::abs(mass-model.caseKg)<=massTolerance);
        const bool rods=model.kind==dmg::Warhead::Kind::Rod;
        REQUIRE(segments==(rods?static_cast<unsigned>(field.projectiles.size()):0u));
        REQUIRE((connected>0)==(mechanism=="native.warhead.connected_ring.v1"));
        dmg::Airframe absorber;dmg::Airframe::Component component;
        component.kind="fuel";component.tank=0;component.half={extent+1,extent+1,extent+1};absorber.components.push_back(component);
        dmg::Target target;target.velocity=targetVelocity;target.airDensity=0;
        target.bodyToEcef={dmg::V3{1,0,0},dmg::V3{0,1,0},dmg::V3{0,0,1}};
        dmg::SceneEnvironmentV1 environment;environment.airDensity=0;
        auto vacuum=field;vacuum.applyBlast=false;
        const auto assessed=dmg::assessSceneV1(vacuum,{{1,dmg::AttachedGeometryView(absorber),target}},environment);
        REQUIRE(assessed.valid);REQUIRE(assessed.victims.size()==1);
        REQUIRE(assessed.victims[0].impact.components.size()==1);
        const auto& absorption=assessed.victims[0].impact.components[0];
        REQUIRE(absorption.hits==model.fragments);REQUIRE(assessed.firstStrikeAgeS==0);
        REQUIRE(std::abs(absorption.energyJ-referenceEnergy)<=energyTolerance);
        const double momentumError=dmg::length(assessed.transferredMomentumEcef-referenceMomentum);
        REQUIRE(momentumError<=momentumTolerance);REQUIRE(std::abs(assessed.absorbedMassKg-mass)<=massTolerance);
        REQUIRE(assessed.remainingMassKg==0);REQUIRE(dmg::length(assessed.airMomentumEcef)==0);
        // A separate real World uses the unchanged selected source, full blast,
        // native atmosphere and maintained F-16 geometry. A fragment Hit AND a
        // tank hole are required: blast-only structural loss cannot pass.
        const auto probe=std::make_shared<Probe>();probe->card=name;session::World world(options());
        REQUIRE(world.setPayloadProvidersV1(observedRegistry(*descriptor,probe),error));
        session::VehicleSpec spec;spec.type="jsbsim:f16c";spec.name="native-component-victim";
        spec.initial.altitudeMslM=6000;spec.initial.airspeedTrueMs=220;spec.initial.headingDeg=90;
        const auto id=world.createVehicle(spec);REQUIRE(id);world.step();REQUIRE(finiteWorld(world));const auto state=*world.vehicleState(id);
        const auto frame=dmg::readAirframe(std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR)/"f16c/f16c.damage.json",error);REQUIRE(frame);
        const auto tank=std::find_if(frame->components.begin(),frame->components.end(),[](const auto& item){return item.kind=="fuel"&&item.tank>=0;});
        REQUIRE(tank!=frame->components.end());double cg[3];REQUIRE(world.model(id)->cgFromOriginBodyM(cg));
        const dmg::V3 at=dmg::V3{state.positionEcef[0],state.positionEcef[1],state.positionEcef[2]}+
            rotate(state,tank->centre-dmg::V3{cg[0],cg[1],cg[2]});
        const auto axis=rotate(state,{1,0,0});const auto velocity=rotate(state,{220,0,0});
        session::Detonation native;native.store=name;
        for(int k=0;k<3;++k){native.positionEcef[k]=(&at.x)[k];native.axisEcef[k]=(&axis.x)[k];native.velocityEcef[k]=(&velocity.x)[k];}
        const auto events=world.detonate(native);unsigned hits=0;
        for(const auto& event:events)if(event.kind==static_cast<std::uint8_t>(ipc::EngagementKind::Hit))hits+=event.index;
        REQUIRE(probe->calls==1);REQUIRE(probe->emitted==field.projectiles.size());REQUIRE(std::abs(probe->mass-mass)<=massTolerance);
        REQUIRE(hits>0);REQUIRE(world.damage(id));double hole=0;
        for(const auto& [index,value]:world.damage(id)->tanks()){(void)index;hole+=value.holeM2;}
        REQUIRE(hole>0);REQUIRE(std::isfinite(hole));const bool nativeFinite=finiteWorld(world);REQUIRE(nativeFinite);
        if(out.is_open())out<<name<<','<<descriptor->implementationId<<','<<mechanism<<','<<model.modelVersion<<','<<segments<<','<<connected<<','<<field.projectiles.size()<<','<<model.caseKg<<','<<mass<<','<<massTolerance<<','<<referenceEnergy<<','<<absorption.energyJ<<','<<energyTolerance<<','<<momentumError<<','<<momentumTolerance<<','<<assessed.absorbedMassKg<<','<<probe->calls<<','<<probe->emitted<<','<<probe->mass<<','<<hits<<','<<hole<<','<<nativeFinite<<','<<model.tntKg<<'\n'<<std::flush;
    }
    REQUIRE(tested==26);
}

TEST_CASE("CJ10 armed native terrain initiation executes its warhead while inert releases remain inert", "[effects-cruise-terminal][scene-effects]") {
    auto out=receipt("cruise-terminal.csv","card,frame_skip,armed,jettisoned,arming_s,impact_age_s,detonations,scene_calls,emitted,emitted_kg,case_kg,terminal,finite_worlds");
    std::vector<std::string> errors;const auto card=ms::readStoreCard(std::filesystem::path(FSIM_TEST_STORE_DIR)/"cj-10a.json",errors);
    REQUIRE(card);REQUIRE(errors.empty());cmp::ProfileV1 profile;std::string error;
    REQUIRE(cmp::readProfileV1(std::filesystem::path(FSIM_TEST_STORE_DIR)/"cj-10a.json",*card,profile,error));
    const auto* descriptor=profile.find("warhead");REQUIRE(descriptor);const auto model=dmg::warheadOf(*card);REQUIRE(model);
    double arming=0;REQUIRE(cmp::parameterV1(*profile.find("fuze"),"arming_s","s",arming,error));
    for(const int skip:{1,4})for(const bool armed:{true,false}) {
        INFO("frameSkip="<<skip<<" armed="<<armed);
        const auto probe=std::make_shared<Probe>();session::World world(options(skip));
        REQUIRE(world.setPayloadProvidersV1(observedRegistry(*descriptor,probe),error));
        session::VehicleSpec aircraft;aircraft.name="carrier";aircraft.type="jsbsim:h6k";
        aircraft.initial.altitudeMslM=6000;aircraft.initial.headingDeg=90;aircraft.initial.airspeedTrueMs=200;
        const auto carrier=world.createVehicle(aircraft);REQUIRE(carrier);
        REQUIRE(world.loadStore(carrier,0,1,"cj-10a")==ms::Reason::None);
        if(armed) {
            // The normal armed Host requires a genuine delivered designation.
            // Reuse the established native donor/link contract, never feed
            // referee target truth into the waypoint provider.
            aircraft.name="sensor donor";aircraft.type="jsbsim:f16c";
            aircraft.initial.latitudeDeg-=.02;
            const auto donor=world.createVehicle(aircraft);REQUIRE(donor);
            aircraft.name="coordinate source";aircraft.initial.longitudeDeg+=.18;
            aircraft.initial.headingDeg=270;const auto target=world.createVehicle(aircraft);REQUIRE(target);
            for(const auto id:{carrier,donor,target}) {
                control::VelocityCommand hold;hold.airspeedMs=200;hold.verticalSpeedMs=0;
                hold.headingRad=(id==target?270.:90.)*std::acos(-1.)/180;
                REQUIRE(world.submit(id,hold).accepted());
            }
            comm::NetSpec net;net.name="native-coordinate-link";net.latencyS=.01;net.jitterS=0;net.dataRateBps=1e6;
            const auto network=world.createNet(net);REQUIRE(network);
            const auto source=world.assignSystem(donor,0x4001,"DONOR");REQUIRE(source);
            REQUIRE(world.joinNet(donor,network));REQUIRE(world.joinNet(carrier,network));
            comm::DisseminationPlan plan;plan.tracksPeriodS=.2;REQUIRE(world.setDissemination(donor,plan));
            REQUIRE(world.submit(donor,ms::StateCommand{0,ms::PayloadState::Operate}).accepted());
            REQUIRE(world.submit(carrier,ms::StateCommand{0,ms::PayloadState::Operate}).accepted());
            ms::AmtiCommand search;search.payload=0;search.azimuthHalfWidthRad=.2;search.bars=1;search.rangeScaleM=60000;
            REQUIRE(world.submit(donor,search).accepted());std::uint32_t designation=0;std::uint64_t observed=0;
            double sent=-std::numeric_limits<double>::infinity();
            for(unsigned k=0;k<4800&&!designation;++k) {
                world.step();REQUIRE(finiteWorld(world));
                for(const auto& track:world.amtiTracks(donor,0))if(track.truth==target&&track.updates>=3)observed=track.id;
                if(observed&&world.simTime()-sent>=.2) {
                    comm::DmsReason why;REQUIRE_FALSE(world.publishNative(donor,comm::topic::kEntity,0,{},why).empty());
                    REQUIRE(why==comm::DmsReason::None);sent=world.simTime();
                }
                if(observed)for(const auto& row:world.dms().linkTracks(carrier))
                    if(row.source==source&&row.report.track==observed&&row.report.payload==0)
                        designation=world.designateLink(carrier,source,0,observed);
            }
            REQUIRE(designation>=session::World::kDesignations);
            ms::StrikeCommand strike;strike.payload=0;strike.store="cj-10a";strike.target=designation;strike.consent=true;
            REQUIRE(world.submit(carrier,strike).accepted());
        } else {
            ms::JettisonCommand command;command.payload=0;command.emergency=true;REQUIRE(world.submit(carrier,command).accepted());
        }
        for(unsigned k=0;k<2400&&world.weapons().empty();++k){world.step();REQUIRE(finiteWorld(world));}
        REQUIRE(world.weapons().size()==1);const auto id=world.weapons().front().id;
        REQUIRE(weapon(world,id).jettisoned==!armed);
        // Independent fixture construction after the unchanged authored arming
        // interval. Reset preserves release/fuze identity, then native dynamics
        // supplies the actual STRUCTURE/terrain contact. No scripted detonation.
        const double step=world.dt()*skip;
        for(unsigned k=0;k<2400&&world.simTime()-weapon(world,id).launchTimeS<arming+.25;++k){world.step();REQUIRE(finiteWorld(world));}
        const double impactAge=world.simTime()-weapon(world,id).launchTimeS;REQUIRE(impactAge>=arming);
        sim::InitialConditions impact;impact.altitudeMslM=2;impact.airspeedTrueMs=0;impact.enginesRunning=false;
        impact.velocityNedMs[0]=0;impact.velocityNedMs[1]=0;impact.velocityNedMs[2]=80;
        REQUIRE(world.resetVehicle(id,&impact));unsigned observations=0;
        for(unsigned k=0;k<static_cast<unsigned>(std::ceil(3/step));++k) {
            world.step();REQUIRE(finiteWorld(world));++observations;
            if(weapon(world,id).end!=ms::WeaponEnd::InFlight)break;
        }
        const auto record=weapon(world,id);unsigned detonations=0;
        for(const auto& event:world.engagementEvents())if(event.entity==id&&event.kind=="detonation")++detonations;
        if(armed) {
            REQUIRE(record.end==ms::WeaponEnd::Detonated);REQUIRE(detonations==1);REQUIRE(probe->calls==1);
            REQUIRE(probe->emitted==static_cast<std::size_t>(model->fragments));
            REQUIRE(std::abs(probe->mass-model->caseKg)<=512*std::numeric_limits<double>::epsilon()*model->fragments*std::max(1.,model->caseKg));
        } else {REQUIRE(record.end!=ms::WeaponEnd::Detonated);REQUIRE(detonations==0);REQUIRE(probe->calls==0);REQUIRE(probe->emitted==0);}
        if(out.is_open())out<<"cj-10a,"<<skip<<','<<armed<<','<<record.jettisoned<<','<<arming<<','<<impactAge<<','<<detonations<<','<<probe->calls<<','<<probe->emitted<<','<<probe->mass<<','<<model->caseKg<<','<<static_cast<int>(record.end)<<','<<observations<<'\n'<<std::flush;
    }
}
