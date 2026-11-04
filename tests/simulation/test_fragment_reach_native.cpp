// Referee-only reach mechanism fixture, not authored-population lethality or
// statistical hit evidence. One prescribed point retains the actual AIM-120C
// fragment mass, area, air Cd and ejection speed without changing its card or
// provider model. Receiver geometry, flight and damage are actual native F-16.
// Geometry is prescribed before outcomes; no direction or seed is re-sampled.
#include "simulation/runtime/world/World.h"
#include "simulation/mission_systems/components/registration/BuiltInProvidersV1.h"
#include "simulation/mission_systems/weapon/MaterialPoint.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace {
using namespace fsim;
namespace damage=ms::damage;
namespace components=ms::components;
using damage::V3;
constexpr double gravityMs2=9.80665,slugFt3ToKgM3=515.378818;
V3 vector(const double* p){return {p[0],p[1],p[2]};}
double coordinate(V3 p,unsigned axis){return axis==0?p.x:(axis==1?p.y:p.z);}
double norm(V3 p){return std::hypot(p.x,p.y,p.z);}
V3 rotate(const sim::VehicleState& s,V3 p) {
    const auto* r=s.rotationBodyToEcef;
    return {r[0]*p.x+r[1]*p.y+r[2]*p.z,r[3]*p.x+r[4]*p.y+r[5]*p.z,r[6]*p.x+r[7]*p.y+r[8]*p.z};
}
session::WorldOptions options(int skip,unsigned workers) {
    static std::atomic<unsigned> sequence{0};session::WorldOptions out;
    out.name="fragment-reach-"+std::to_string(sequence.fetch_add(1));out.jsbsimRoot=FSIM_TEST_JSBSIM_ROOT;
    out.seed=17;out.frameSkip=skip;out.workers=workers;out.pinWorkers=false;out.publish=false;return out;
}
session::VehicleSpec f16(std::string name) {
    session::VehicleSpec out;out.name=std::move(name);out.type="jsbsim:f16c";
    out.initial.altitudeMslM=6000;out.initial.airspeedTrueMs=220;out.initial.headingDeg=90;return out;
}
struct Probe {unsigned emissions=0,legacyCalls=0;};
class Point final:public damage::WarheadAdapter,public damage::WarheadSceneExecutionV2 {
public:
    Point(damage::Warhead model,std::string id,std::shared_ptr<Probe> probe):model_(std::move(model)),id_(std::move(id)),probe_(std::move(probe)){}
    std::string_view implementationId()const noexcept override{return id_;}
    const damage::Warhead& model()const noexcept override{return model_;}
    damage::Impact assess(const damage::Burst&,const damage::AttachedGeometryView&,const damage::Target&,Rng&)const override {
        FAIL("Shared point fixture must execute the physical scene");return {};
    }
    damage::FragmentFieldV1 emitScene(const damage::Burst& burst,Rng&)const override {
        ++probe_->emissions;damage::FragmentFieldV1 field;field.valid=true;field.burst=burst;
        const double mass=model_.caseKg/static_cast<double>(model_.fragments);
        field.caseKg=mass;field.applyBlast=false;field.emittedCount=1;
        damage::FragmentFieldV1::Projectile p;p.sourceOrdinalV1=1;p.massKg=mass;
        p.areaM2=model_.fragmentAreaM2;p.cd=model_.fragmentCd;p.at=burst.at;
        p.velocity=burst.velocity+damage::normalized(burst.axis)*model_.speedMs;
        field.projectiles.push_back(p);return field;
    }
private:
    damage::Warhead model_;std::string id_;std::shared_ptr<Probe> probe_;
};
// This intentionally external, non-scene adapter only observes legacy dispatch.
// Its stipulated response is not a physical fragment or lethality reference.
class External final:public damage::WarheadAdapter {
public:
    External(damage::Warhead model,std::string id,std::shared_ptr<Probe> probe):model_(std::move(model)),id_(std::move(id)),probe_(std::move(probe)){}
    std::string_view implementationId()const noexcept override{return id_;}
    const damage::Warhead& model()const noexcept override{return model_;}
    damage::Impact assess(const damage::Burst&,const damage::AttachedGeometryView&,const damage::Target&,Rng&)const override {
        ++probe_->legacyCalls;damage::Impact out;out.fragmentsThrown=1;out.fragmentsStriking=1;out.impulseBody={.01,0,0};return out;
    }
private:
    damage::Warhead model_;std::string id_;std::shared_ptr<Probe> probe_;
};
struct Selection {damage::Warhead model;components::ProviderRegistryV1 registry;};
Selection selection(std::shared_ptr<Probe> probe,bool scene=true) {
    const auto path=std::filesystem::path(FSIM_TEST_STORE_DIR)/"aim-120c.json";
    std::vector<std::string> errors;const auto card=ms::readStoreCard(path,errors);REQUIRE(card);REQUIRE(errors.empty());
    components::ProfileV1 profile;std::string error;REQUIRE(components::readProfileV1(path,*card,profile,error));
    const auto* descriptor=profile.find("warhead");REQUIRE(descriptor);
    auto registry=components::builtInProvidersV1();const auto original=registry.warhead;
    components::ProviderContextV1 context{*card,profile,{},std::nullopt};
    const auto authored=original.create(*descriptor,context,error);INFO(error);REQUIRE(authored);
    const auto model=authored->model();REQUIRE(model.fragments>0);REQUIRE(model.caseKg>0);
    REQUIRE(model.fragmentAreaM2>0);REQUIRE(model.fragmentCd>0);REQUIRE(model.speedMs>0);
    REQUIRE(registry.warhead.registerFactory(descriptor->implementationId,std::string(equipment::mechanism(descriptor->implementationId)),
        [original,probe,scene](const auto& row,const auto& ctx,auto& why)->std::unique_ptr<damage::WarheadAdapter> {
            const auto inner=original.create(row,ctx,why);if(!inner)return {};
            if(scene)return std::make_unique<Point>(inner->model(),std::string(inner->implementationId()),probe);
            return std::make_unique<External>(inner->model(),std::string(inner->implementationId()),probe);
        },true,error));return {model,std::move(registry)};
}
damage::Airframe frame() {
    std::string error;const auto value=damage::readAirframe(std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR)/"f16c/f16c.damage.json",error);
    INFO(error);REQUIRE(value);REQUIRE(value->contactGeometry);return *value;
}
V3 enginePoint(const damage::Airframe& airframe) {
    const auto found=std::find_if(airframe.components.begin(),airframe.components.end(),[](const auto& c){return c.kind=="engine";});
    REQUIRE(found!=airframe.components.end());REQUIRE(std::min({found->half.x,found->half.y,found->half.z})>0);return found->centre;
}
V3 cg(session::World& world,std::uint32_t id) {
    double value[3]{};REQUIRE(world.cgFromOriginBodyM(id,value));return vector(value);
}
double radius(const damage::Airframe& airframe,V3 centre) {
    double result=0;
    for(std::size_t i=0;i<airframe.panels.size();++i) {
        if(const auto* mesh=airframe.contactGeometry->panel(i))for(const auto p:mesh->vertices)result=std::max(result,norm(p-centre));
        else for(const auto p:airframe.panels[i].corners)result=std::max(result,norm(p-centre));
    }
    for(std::size_t i=0;i<airframe.components.size();++i) {
        const auto& c=airframe.components[i];result=std::max(result,norm(c.centre-centre)+norm(c.half));
        if(const auto* mesh=airframe.contactGeometry->component(i))for(const auto p:mesh->vertices)result=std::max(result,norm(p-centre));
    }
    REQUIRE(result>0);return result;
}
sim::NativePositionV1 nativePosition(session::World& world,std::uint32_t id,V3 at) {
    const auto* source=dynamic_cast<const sim::NativePositionSourceV1*>(world.model(id));REQUIRE(source);
    const double value[]{at.x,at.y,at.z};sim::NativePositionV1 out;REQUIRE(source->nativePositionV1(value,out));return out;
}
V3 gravity(const sim::NativePositionV1& p) {
    return {-gravityMs2*std::cos(p.latitudeRad)*std::cos(p.longitudeRad),
        -gravityMs2*std::cos(p.latitudeRad)*std::sin(p.longitudeRad),-gravityMs2*std::sin(p.latitudeRad)};
}
double airDensity(session::World& world,std::uint32_t id) {
    const auto value=world.model(id)->property("atmosphere/rho-slugs_ft3");REQUIRE(value.valid());
    const double density=value.get()*slugFt3ToKgM3;REQUIRE(std::isfinite(density));REQUIRE(density>0);return density;
}
struct Flight {V3 at,velocity;double distance=0;};
// Independent continuous ODE oracle. This does not call fragmentFlight,
// assessScene, liquid transit, or the native fragment integrator.
Flight freePoint(Flight p,double time,double coefficient,V3 g,double maxStep=1./24000) {
    const unsigned count=static_cast<unsigned>(std::ceil(time/maxStep));REQUIRE(count>0);
    const double h=time/static_cast<double>(count);
    const auto acceleration=[&](V3 v){return g-v*(coefficient*norm(v));};
    for(unsigned i=0;i<count;++i) {
        const V3 a=acceleration(p.velocity),vb=p.velocity+a*(h*.5),b=acceleration(vb);
        const V3 vc=p.velocity+b*(h*.5),c=acceleration(vc),vd=p.velocity+c*h,d=acceleration(vd);
        p.distance+=(norm(p.velocity)+2*norm(vb)+2*norm(vc)+norm(vd))*(h/6);
        p.at+=(p.velocity+vb*2+vc*2+vd)*(h/6);p.velocity+=(a+b*2+c*2+d)*(h/6);
    }
    return p;
}
double coefficient(const damage::Warhead& w,double density) {
    return .5*density*w.fragmentCd*w.fragmentAreaM2/(w.caseKg/static_cast<double>(w.fragments));
}
session::Detonation burst(V3 at,V3 velocity,V3 direction,std::uint32_t source=0) {
    session::Detonation out;out.store="aim-120c";out.weapon=source;
    for(unsigned i=0;i<3;++i){out.positionEcef[i]=coordinate(at,i);out.velocityEcef[i]=coordinate(velocity,i);out.axisEcef[i]=coordinate(direction,i);}
    if(source)out.stepFraction=1;
    return out;
}
bool isHit(const ipc::EngagementEvent& e,std::uint32_t slot) {
    return e.slot==slot&&e.kind==static_cast<std::uint8_t>(ipc::EngagementKind::Hit)&&e.index>0;
}
using Signature=std::vector<std::array<double,11>>;
Signature hitSignature(const std::vector<ipc::EngagementEvent>& events,std::uint32_t slot) {
    Signature out;
    for(const auto& e:events)if(isHit(e,slot))out.push_back({e.simTime,static_cast<double>(e.slot),static_cast<double>(e.target),
        static_cast<double>(e.index),e.positionEcef[0],e.positionEcef[1],e.positionEcef[2],e.velocityEcef[0],e.velocityEcef[1],e.velocityEcef[2],e.massKg});
    return out;
}
enum class Path {Hit,Miss,TooLate};
Signature synchronous(Path path,unsigned workers) {
    const auto probe=std::make_shared<Probe>();const auto selected=selection(probe);session::World world(options(1,workers));
    std::string error;REQUIRE(world.setPayloadProvidersV1(selected.registry,error));const auto victim=world.createVehicle(f16("receiver"));REQUIRE(victim);world.step(2);
    const auto airframe=frame();const auto state=*world.vehicleState(victim);const auto point=ms::weapon::materialPoint(state,cg(world,victim),enginePoint(airframe));REQUIRE(point.valid);
    const V3 velocity=rotate(state,vector(state.velocityBodyMs)),direction=rotate(state,{0,1,0});
    const double time=path==Path::TooLate?1.6:.6,k=coefficient(selected.model,airDensity(world,victim));
    INFO("diagnostic authored point mass="<<selected.model.caseKg/static_cast<double>(selected.model.fragments)
        <<" area="<<selected.model.fragmentAreaM2<<" Cd="<<selected.model.fragmentCd<<" ejection speed="<<selected.model.speedMs);
    const V3 initialVelocity=velocity+direction*selected.model.speedMs;
    V3 at=point.position;V3 g;
    // Resolve the declared frozen gravity at the prescribed source using the
    // actual native planet conversion, rather than a guessed ellipsoid.
    for(unsigned i=0;i<3;++i){g=gravity(nativePosition(world,victim,at));at=point.position+point.velocity*time-freePoint({{},initialVelocity},time,k,g).at;}
    if(path==Path::Miss)at+=rotate(state,{0,0,60});
    REQUIRE(norm(at-vector(state.positionEcef))>150);
    const auto fine=freePoint({at,initialVelocity},time,k,g),coarse=freePoint({at,initialVelocity},time,k,g,1./12000);
    REQUIRE(norm(fine.at-coarse.at)<1e-4);
    if(path!=Path::Hit) {
        const double bound=radius(airframe,cg(world,victim));double previousDistance=0;
        for(unsigned i=1;i<=120;++i){const double t=static_cast<double>(i)/120;
            const auto p=freePoint({at,initialVelocity},t,k,g);
            const double swept=p.distance-previousDistance+norm(velocity)/120;
            REQUIRE(norm(p.at-vector(state.positionEcef)-velocity*t)>bound+swept+5);previousDistance=p.distance;}
    }
    const auto events=world.detonate(burst(at,velocity,direction));const auto slot=world.vehicleStreamSlot(victim);REQUIRE(slot);
    const auto hits=hitSignature(events,*slot);
    if(path==Path::Hit){CHECK(probe->emissions==1);CHECK_FALSE(hits.empty());}
    else CHECK(hits.empty());
    CHECK(probe->legacyCalls==0);return hits;
}
struct Forecast {V3 materialDisplacement;std::vector<V3> cgDisplacements;double maxSpeed=0;};
Forecast forecast(int skip,double duration,V3 material) {
    session::World world(options(skip,1));const auto id=world.createVehicle(f16("undamaged forecast"));REQUIRE(id);
    const V3 start=vector(world.vehicleState(id)->positionEcef);Forecast out;out.cgDisplacements.push_back({});
    out.maxSpeed=norm(rotate(*world.vehicleState(id),vector(world.vehicleState(id)->velocityBodyMs)));
    const double dt=world.dt()*static_cast<double>(world.frameSkip());const auto steps=static_cast<unsigned>(std::llround(duration/dt));
    REQUIRE(std::abs(static_cast<double>(steps)*dt-duration)<1e-12);
    for(unsigned i=0;i<steps;++i){world.step();const auto& state=*world.vehicleState(id);
        out.cgDisplacements.push_back(vector(state.positionEcef)-start);out.maxSpeed=std::max(out.maxSpeed,norm(rotate(state,vector(state.velocityBodyMs))));}
    const auto point=ms::weapon::materialPoint(*world.vehicleState(id),cg(world,id),material);REQUIRE(point.valid);
    out.materialDisplacement=point.position-start;return out;
}
Signature causal(int skip,unsigned workers,bool movingOut,Path path) {
    const auto airframe=frame();const double intercept=path==Path::TooLate?1.6:(movingOut?.8:.6);
    // This undamaged native forecast is fixed before emission and contains no
    // damage outcomes. It only prescribes receiver initial geometry; none of
    // this referee data is fed to a sensor, guidance provider or controller.
    const auto reference=forecast(skip,intercept,enginePoint(airframe));
    const auto probe=std::make_shared<Probe>();const auto selected=selection(probe);session::World world(options(skip,workers));
    std::string error;REQUIRE(world.setPayloadProvidersV1(selected.registry,error));
    const auto launcher=world.createVehicle(f16("actual launcher"));REQUIRE(launcher);
    ms::WeaponLaunch release;release.type="jsbsim:aim120c";release.offsetBodyM[1]=30;
    const auto source=world.launchWeapon(launcher,release);REQUIRE(source);world.step(2);
    const auto sourceState=*world.vehicleState(source);const V3 at=vector(sourceState.positionEcef);
    const V3 velocity=rotate(sourceState,vector(sourceState.velocityBodyMs)),direction=rotate(sourceState,movingOut?V3{1,0,0}:V3{0,1,0});
    const double k=coefficient(selected.model,airDensity(world,source));const V3 g=gravity(nativePosition(world,source,at));
    const V3 initialVelocity=velocity+direction*selected.model.speedMs;
    const auto predicted=freePoint({at,initialVelocity},intercept,k,g),coarse=freePoint({at,initialVelocity},intercept,k,g,1./12000);
    REQUIRE(norm(predicted.at-coarse.at)<1e-4);
    V3 receiverAt=predicted.at-reference.materialDisplacement;
    if(path==Path::Miss)receiverAt+=rotate(sourceState,{0,0,60});
    const auto location=nativePosition(world,source,receiverAt);auto spec=f16("actual receiver");const double degrees=180/std::acos(-1.);
    spec.initial.latitudeDeg=location.latitudeRad*degrees;spec.initial.longitudeDeg=location.longitudeRad*degrees;spec.initial.altitudeMslM=location.altitudeMslM;
    const auto victim=world.createVehicle(spec);REQUIRE(victim);const V3 actualAt=vector(world.vehicleState(victim)->positionEcef);
    REQUIRE(norm(actualAt-receiverAt)<.01);const double startingRange=norm(actualAt-at);
    INFO("frame skip="<<skip<<" workers="<<workers<<" moving out="<<movingOut<<" prescribed time="<<intercept<<" starting range="<<startingRange);
    if(path==Path::Hit){if(movingOut)REQUIRE(startingRange<150-5);else REQUIRE(startingRange>150+5);}
    const double dt=world.dt()*static_cast<double>(world.frameSkip());
    if(path==Path::Hit&&movingOut) {
        // Independently rule out an earlier near-source interception: this
        // geometry must first clear the legacy radius before material contact,
        // including the receiver's entire conservative authored-geometry ball.
        const double bound=radius(airframe,cg(world,victim));double previousDistance=0;
        for(std::size_t i=0;i<reference.cgDisplacements.size();++i) {
            const double t=static_cast<double>(i)*dt;const V3 centre=actualAt+reference.cgDisplacements[i];
            const auto point=t==0?Flight{at,initialVelocity}:freePoint({at,initialVelocity},t,k,g);
            const double swept=point.distance-previousDistance+(reference.maxSpeed+10)*dt;
            if(norm(centre-at)<=150+(reference.maxSpeed+10)*dt)REQUIRE(norm(point.at-centre)>bound+swept+1e-3);
            previousDistance=point.distance;
        }
    }
    if(path!=Path::Hit) {
        const double bound=radius(airframe,cg(world,victim));double previousDistance=0;
        for(std::size_t i=1;i<reference.cgDisplacements.size();++i){const double t=static_cast<double>(i)*dt;if(t>1)break;
            const auto p=freePoint({at,initialVelocity},t,k,g);const double swept=p.distance-previousDistance+(reference.maxSpeed+10)*dt;
            REQUIRE(norm(p.at-actualAt-reference.cgDisplacements[i])>bound+swept+5);previousDistance=p.distance;}
    }
    const auto sourceSlot=world.vehicleStreamSlot(source),victimSlot=world.vehicleStreamSlot(victim);REQUIRE(sourceSlot);REQUIRE(victimSlot);
    const auto immediate=world.detonateCapturedV1(burst(at,velocity,direction,source));CHECK(probe->emissions==1);
    CHECK(hitSignature(immediate,*victimSlot).empty());const double epoch=world.simTime();
    REQUIRE(world.removeVehicle(source));session::VehicleSpec replacement;replacement.name="unrelated slot replacement";replacement.type="jsbsim:aim120c";
    replacement.initial=world.info(launcher)->initial;replacement.initial.longitudeDeg+=.1;
    const auto other=world.createVehicle(replacement);REQUIRE(other);CHECK(world.vehicleStreamSlot(other)==sourceSlot);
    world.takeEngagementEvents();std::vector<ipc::EngagementEvent> all;
    const auto steps=static_cast<unsigned>(std::ceil(1.1/dt));
    for(unsigned i=0;i<steps;++i){world.step();const auto events=world.takeEngagementEvents();
        for(const auto& event:events)if(isHit(event,*victimSlot)) {
            CHECK(event.target==*sourceSlot+1);CHECK(event.simTime>epoch);CHECK(event.simTime<=epoch+1+dt);
            CHECK(std::string(event.name)=="aim-120c");CHECK(norm(vector(event.positionEcef)-at)>150);
        }
        all.insert(all.end(),events.begin(),events.end());
    }
    const auto hits=hitSignature(all,*victimSlot);
    if(path==Path::Hit)CHECK_FALSE(hits.empty());else CHECK(hits.empty());
    const auto outcomes=world.engagementEvents();
    CHECK(std::none_of(outcomes.begin(),outcomes.end(),[&](const auto& e){return e.kind=="damage"&&e.entity==other;}));
    if(path==Path::Hit)CHECK(std::any_of(outcomes.begin(),outcomes.end(),[&](const auto& e){return e.kind=="damage"&&e.entity==source&&e.launcher==launcher&&e.target==victim;}));
    CHECK(probe->legacyCalls==0);return hits;
}
}

TEST_CASE("native shared fragment field reaches actual F16 beyond the legacy burst radius", "[damage][world][fragment-reach][fragment-reach-synchronous]") {
    for(const auto path:{Path::Hit,Path::Miss,Path::TooLate})CHECK(synchronous(path,1)==synchronous(path,2));
}

TEST_CASE("native causal fragment retains reachable victims after original source removal", "[damage][world][fragment-reach][fragment-reach-causal]") {
    for(const int skip:{1,4})for(const bool movingOut:{false,true}) {
        INFO("frame skip="<<skip<<" moving out="<<movingOut);CHECK(causal(skip,1,movingOut,Path::Hit)==causal(skip,2,movingOut,Path::Hit));
    }
    CHECK(causal(1,1,false,Path::Miss).empty());CHECK(causal(1,1,false,Path::TooLate).empty());
}

TEST_CASE("native external warhead adapter preserves its established 150 metre dispatch limit", "[damage][world][fragment-reach][fragment-reach-legacy]") {
    for(const double range:{100.,200.}) {
        const auto probe=std::make_shared<Probe>();const auto selected=selection(probe,false);session::World world(options(1,1));
        std::string error;REQUIRE(world.setPayloadProvidersV1(selected.registry,error));const auto victim=world.createVehicle(f16("legacy receiver"));REQUIRE(victim);world.step(2);
        const auto state=*world.vehicleState(victim);const auto events=world.detonate(burst(vector(state.positionEcef)+rotate(state,{0,range,0}),{},rotate(state,{1,0,0})));
        const auto slot=world.vehicleStreamSlot(victim);REQUIRE(slot);CHECK(probe->legacyCalls==(range<150?1u:0u));CHECK(probe->emissions==0);
        CHECK(hitSignature(events,*slot).empty()==(range>=150));
    }
}
