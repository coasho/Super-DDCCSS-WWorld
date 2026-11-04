#include "simulation/mission_systems/damage/LiquidSceneV1.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace fsim::ms::damage;
namespace {
Target liquidTarget(V3 origin={},V3 velocity={}) {
    Target t;t.at=origin;t.velocity=velocity;t.airDensity=0;
    t.bodyToEcef={V3{1,0,0},V3{0,1,0},V3{0,0,1}};return t;
}
Airframe liquidFrame(bool downstream=false) {
    Airframe f;f.sections.resize(1);
    Airframe::Component tank;tank.kind="fuel";tank.tank=0;tank.section=0;tank.centre={.5,0,0};tank.half={.5,1,1};f.components.push_back(tank);
    if(downstream){Airframe::Component item;item.kind="engine";item.section=0;item.centre={2,0,0};item.half={.05,.5,.5};f.components.push_back(item);}
    return f;
}
SceneLiquidInputsV1 liquidInputs(double fraction=1,unsigned subdivisions=1) {
    SceneLiquidInputsV1 inputs;SceneLiquidRecipientV1 r;r.identity=7;r.generation=3;
    r.tanks.push_back({0,1000,.2,4000*fraction,{{0,1,0},-1+2*fraction}});
    for(unsigned i=0;i<subdivisions;++i)r.cells.push_back({static_cast<std::uint64_t>(i)+1,0,0,0,
        {{static_cast<double>(i)/subdivisions,-1,-1},{static_cast<double>(i+1)/subdivisions,1,1}}});
    inputs.recipients.push_back(std::move(r));return inputs;
}
FragmentFieldV1 liquidPoint(V3 origin={-1,0,0},V3 velocity={100,0,0}) {
    FragmentFieldV1 f;f.valid=true;f.caseKg=1;f.applyBlast=false;
    FragmentFieldV1::Projectile p;p.at=origin;p.velocity=velocity;p.massKg=1;p.areaM2=.001;p.cd=1.3;f.projectiles.push_back(p);return f;
}
SceneEnvironmentV1 liquidEnvironment(double duration=.05) {
    SceneEnvironmentV1 e;e.airDensity=0;e.maxFlightTimeS=duration;return e;
}
const Impact::ComponentHits* componentHit(const SceneAssessmentV1& r,int index) {
    const auto& hits=r.victims.front().impact.components;
    const auto found=std::find_if(hits.begin(),hits.end(),[&](const auto& h){return h.component==index;});
    return found==hits.end()?nullptr:&*found;
}
const Impact::ComponentHits& requiredComponent(const SceneAssessmentV1& result,int index) {
    const auto* hit=componentHit(result,index);
    if(!hit)throw std::runtime_error("Required liquid component impact is absent");
    return *hit;
}
SceneAssessmentV1 liquidRun(const Airframe& frame,const FragmentFieldV1& field,const SceneEnvironmentV1& environment,
                           const SceneLiquidInputsV1& inputs,Target target=liquidTarget()) {
    return assessSceneLiquidsV1(field,{{7,AttachedGeometryView(frame),target,3}},environment,inputs);
}
void near(double actual,double reference,double tolerance) {INFO("actual="<<actual<<" reference="<<reference<<" tolerance="<<tolerance);CHECK(std::abs(actual-reference)<=tolerance);}
void noBladderPuncture(const SceneAssessmentV1& result) {
    REQUIRE(result.valid);REQUIRE(result.victims.size()==1);
    for(const auto& hit:result.victims[0].impact.components){CHECK(hit.hits==0);CHECK(hit.areaM2==0);}
}
}

TEST_CASE("interior point and rod liquid residence cannot create a bladder puncture", "[damage][liquid-scene][liquid-bladder-boundary]") {
    const auto frame=liquidFrame();
    for(const bool rod:{false,true})for(const bool wet:{false,true}) {
        INFO("rod="<<rod<<" wet="<<wet);auto field=liquidPoint({.25,-.5,0});auto& p=field.projectiles[0];
        p.segment=rod;p.end={.25,.5,0};p.endVelocity=p.velocity;
        const auto result=liquidRun(frame,field,liquidEnvironment(1e-7),liquidInputs(wet?1:0));
        noBladderPuncture(result);REQUIRE(result.continuation.projectiles.size()==1);
        if(wet){REQUIRE(componentHit(result,0));CHECK(requiredComponent(result,0).energyJ>0);CHECK(result.transferredMomentumEcef.x>0);}
        else {CHECK(result.victims[0].impact.components.empty());CHECK(length(result.transferredMomentumEcef)==0);}
        near(result.remainingMassKg,1,1e-14);CHECK(result.absorbedMassKg==0);
    }
}

TEST_CASE("bladder puncture retains actual outside entry and initial true-face contact", "[damage][liquid-scene][liquid-bladder-boundary]") {
    const auto frame=liquidFrame();const auto inputs=liquidInputs(0);
    const auto entry=liquidRun(frame,liquidPoint(),liquidEnvironment(.0101),inputs);REQUIRE(entry.valid);
    REQUIRE(componentHit(entry,0));CHECK(requiredComponent(entry,0).hits==1);near(requiredComponent(entry,0).areaM2,.001,1e-15);
    REQUIRE(entry.firstStrikeAgeS);near(*entry.firstStrikeAgeS,.01,1e-12);
    for(const bool rod:{false,true}) {
        INFO("initial true-face rod="<<rod);auto field=liquidPoint({0,0,0},{-100,0,0});auto& p=field.projectiles[0];
        if(rod){p.segment=true;p.at={-.1,0,0};p.end={.1,0,0};p.velocity=p.endVelocity={0,0,100};}
        const auto result=liquidRun(frame,field,liquidEnvironment(1e-7),inputs);REQUIRE(result.valid);
        REQUIRE(componentHit(result,0));CHECK(requiredComponent(result,0).hits==1);near(requiredComponent(result,0).areaM2,.001,1e-15);
        REQUIRE(result.firstStrikeAgeS);CHECK(*result.firstStrikeAgeS==0);CHECK(requiredComponent(result,0).energyJ==0);
    }
}

TEST_CASE("interior point punctures only at its real later exit across split windows", "[damage][liquid-scene][liquid-bladder-boundary]") {
    const auto frame=liquidFrame();const auto field=liquidPoint({.25,.2,0});
    for(const bool wet:{false,true}) {
        INFO("wet="<<wet);const auto inputs=liquidInputs(wet?1:0);
        const double exitTime=wet?std::expm1(.1*.75)/(.1*100):.75/100;
        const auto before=liquidRun(frame,field,liquidEnvironment(exitTime-1e-6),inputs);noBladderPuncture(before);
        const auto after=liquidRun(frame,before.continuation,liquidEnvironment(2e-6),inputs);REQUIRE(after.valid);
        REQUIRE(componentHit(after,0));CHECK(requiredComponent(after,0).hits==1);near(requiredComponent(after,0).areaM2,.001,1e-15);
        if(!wet){REQUIRE(after.firstStrikeAgeS);near(*after.firstStrikeAgeS,exitTime,1e-12);}
        const auto later=liquidRun(frame,after.continuation,liquidEnvironment(.001),inputs);noBladderPuncture(later);
    }
}

TEST_CASE("a rotating zero-COM-speed rod retains its actual initial bladder intersection", "[damage][liquid-scene][liquid-bladder-boundary]") {
    const auto frame=liquidFrame();auto field=liquidPoint({-.1,0,0},{0,0,-100});auto& p=field.projectiles[0];
    p.segment=true;p.end={.1,0,0};p.endVelocity={0,0,100};
    const auto result=liquidRun(frame,field,liquidEnvironment(1e-7),liquidInputs(0));REQUIRE(result.valid);
    REQUIRE(componentHit(result,0));CHECK(requiredComponent(result,0).hits==1);
    near(requiredComponent(result,0).areaM2,.001,1e-15);near(requiredComponent(result,0).fastestMs,100,1e-12);
    CHECK(requiredComponent(result,0).energyJ==0);REQUIRE(result.firstStrikeAgeS);CHECK(*result.firstStrikeAgeS==0);
    near(result.remainingMassKg,1,1e-14);CHECK(result.absorbedMassKg==0);
    near(length(result.transferredMomentumEcef),0,1e-14);
}

TEST_CASE("interior rotating liquid material is a real encounter without a bladder puncture", "[damage][liquid-scene][liquid-bladder-boundary]") {
    const auto frame=liquidFrame();auto field=liquidPoint({.25,-.5,0},{0,0,-100});auto& p=field.projectiles[0];
    p.segment=true;p.end={.25,.5,0};p.endVelocity={0,0,100};
    // Also exercise a positive interval below endpoint velocity ULP response.
    // Work/torque remain representable; encounter availability must not rely
    // on a floating cancellation residue in the odd net-force integral.
    for(const double dt:{1e-7,std::ldexp(1.,-60)}) {
        INFO("elapsed="<<dt);const auto result=liquidRun(frame,field,liquidEnvironment(dt),liquidInputs());noBladderPuncture(result);
        REQUIRE(componentHit(result,0));CHECK(requiredComponent(result,0).energyJ>0);near(requiredComponent(result,0).energyJ,25000*dt,2e-7);
        CHECK(result.victims[0].impact.angularImpulseBody.x>0);near(result.victims[0].impact.angularImpulseBody.x,125*dt,2e-9);
        near(length(result.transferredMomentumEcef),0,1e-12);
        CHECK(result.victims[0].impact.fragmentsStriking==1);CHECK(result.projectilesStriking==1);
        REQUIRE(result.firstStrikeAgeS);CHECK(*result.firstStrikeAgeS==0);
    }
}

TEST_CASE("adjacent capacity seams never puncture an interior point or spanning rod", "[damage][liquid-scene][liquid-bladder-boundary]") {
    const auto frame=liquidFrame();
    for(const unsigned cells:{1u,2u,127u})for(const bool rod:{false,true}) {
        INFO("cells="<<cells<<" rod="<<rod);auto field=liquidPoint({.5,0,0});auto& p=field.projectiles[0];
        if(rod){p.segment=true;p.at={.4,0,0};p.end={.6,0,0};p.velocity=p.endVelocity={0,100,0};}
        const auto result=liquidRun(frame,field,liquidEnvironment(1e-7),liquidInputs(1,cells));noBladderPuncture(result);
        REQUIRE(componentHit(result,0));CHECK(requiredComponent(result,0).energyJ>0);
    }
}

TEST_CASE("partly covered capacity face preserves its exposed patch and hides its internal patch", "[damage][liquid-scene][liquid-bladder-boundary]") {
    const auto frame=liquidFrame();auto inputs=liquidInputs(0);auto& cells=inputs.recipients[0].cells;
    cells.push_back({2,0,0,0,{{1,-1,-1},{2,0,1}}});
    const auto covered=liquidRun(frame,liquidPoint({.5,-.5,0}),liquidEnvironment(.006),inputs);noBladderPuncture(covered);
    const auto exposed=liquidRun(frame,liquidPoint({.5,.5,0}),liquidEnvironment(.006),inputs);REQUIRE(exposed.valid);
    REQUIRE(componentHit(exposed,0));CHECK(requiredComponent(exposed,0).hits==1);REQUIRE(exposed.firstStrikeAgeS);near(*exposed.firstStrikeAgeS,.005,1e-12);
    auto field=liquidPoint({.5,-.75,0});auto& p=field.projectiles[0];p.segment=true;p.end={.5,.75,0};p.endVelocity=p.velocity;
    // The lower material portion stays in the neighbor while the upper portion
    // crosses the true exposed face; union time occupancy cannot stand in for
    // an actual finite-line surface test.
    const auto before=liquidRun(frame,field,liquidEnvironment(.0049),inputs);noBladderPuncture(before);
    const auto after=liquidRun(frame,field,liquidEnvironment(.0051),inputs);REQUIRE(after.valid);
    REQUIRE(componentHit(after,0));CHECK(requiredComponent(after,0).hits==1);REQUIRE(after.firstStrikeAgeS);near(*after.firstStrikeAgeS,.005,1e-12);
}

TEST_CASE("disconnected regions of one native tank preserve real exit entry and dry gaps", "[damage][liquid-scene][liquid-bladder-boundary]") {
    const auto frame=liquidFrame();auto inputs=liquidInputs(0,2);auto& cells=inputs.recipients[0].cells;
    cells[0].bodyBounds.high.x=.4;cells[1].bodyBounds.low.x=.6;
    const auto gap=liquidRun(frame,liquidPoint({.5,0,0}),liquidEnvironment(.0005),inputs);noBladderPuncture(gap);
    const auto entry=liquidRun(frame,liquidPoint({.5,0,0}),liquidEnvironment(.0011),inputs);REQUIRE(entry.valid);
    REQUIRE(componentHit(entry,0));CHECK(requiredComponent(entry,0).hits==1);REQUIRE(entry.firstStrikeAgeS);near(*entry.firstStrikeAgeS,.001,1e-12);
    const auto inside=liquidPoint({.2,0,0});const auto before=liquidRun(frame,inside,liquidEnvironment(.0019),inputs);noBladderPuncture(before);
    const auto exit=liquidRun(frame,inside,liquidEnvironment(.0021),inputs);REQUIRE(exit.valid);
    REQUIRE(componentHit(exit,0));CHECK(requiredComponent(exit,0).hits==1);REQUIRE(exit.firstStrikeAgeS);near(*exit.firstStrikeAgeS,.002,1e-12);
    auto rod=liquidPoint({.3,0,0},{0,100,0});rod.projectiles[0].segment=true;
    rod.projectiles[0].end={.7,0,0};rod.projectiles[0].endVelocity=rod.projectiles[0].velocity;
    const auto straddle=liquidRun(frame,rod,liquidEnvironment(1e-7),inputs);REQUIRE(straddle.valid);
    REQUIRE(componentHit(straddle,0));CHECK(requiredComponent(straddle,0).hits==1);
    REQUIRE(straddle.firstStrikeAgeS);CHECK(*straddle.firstStrikeAgeS==0);
}

TEST_CASE("liquid scene retains analytic residence time and downstream residual energy", "[damage][liquid-scene]") {
    const auto frame=liquidFrame(true);const auto inputs=liquidInputs();
    const auto field=liquidPoint();const double k=.1,speed=100*std::exp(-k);
    const double wetTime=std::expm1(k)/(k*100),contactTime=.01+wetTime+.95/speed;
    const auto before=liquidRun(frame,field,liquidEnvironment(contactTime-1e-6),inputs);
    REQUIRE(before.valid);CHECK(componentHit(before,1)==nullptr);REQUIRE(before.continuation.projectiles.size()==1);
    near(before.continuation.projectiles[0].velocity.x,speed,1e-7);
    const auto after=liquidRun(frame,field,liquidEnvironment(contactTime+1e-6),inputs);
    REQUIRE(after.valid);REQUIRE(componentHit(after,0));REQUIRE(componentHit(after,1));
    CHECK(requiredComponent(after,0).hits==1);near(requiredComponent(after,0).areaM2,.001,1e-15);
    near(requiredComponent(after,0).energyJ,.5*(10000-speed*speed),1e-5);
    near(requiredComponent(after,1).energyJ,.5*speed*speed,1e-5);
    near(after.absorbedMassKg,1,1e-14);near(after.remainingMassKg,0,1e-14);
    near(after.transferredMomentumEcef.x,100,1e-7);near(after.airMomentumEcef.x,0,1e-14);
    REQUIRE(after.firstStrikeAgeS);near(*after.firstStrikeAgeS,.01,1e-10);
}

TEST_CASE("liquid scene deposits no future transit and continuation adds only elapsed work", "[damage][liquid-scene]") {
    const auto frame=liquidFrame(true);const auto inputs=liquidInputs();const auto field=liquidPoint();
    const auto whole=liquidRun(frame,field,liquidEnvironment(.015),inputs);
    const auto first=liquidRun(frame,field,liquidEnvironment(.0125),inputs);
    REQUIRE(whole.valid);REQUIRE(first.valid);REQUIRE(first.continuation.projectiles.size()==1);
    const auto second=liquidRun(frame,first.continuation,liquidEnvironment(.0025),inputs);REQUIRE(second.valid);
    REQUIRE(second.continuation.projectiles.size()==1);const double referenceSpeed=100/1.05;
    near(whole.continuation.projectiles[0].at.x,std::log1p(.05)/.1,1e-8);
    near(whole.continuation.projectiles[0].velocity.x,referenceSpeed,1e-8);
    near(second.continuation.projectiles[0].at.x,whole.continuation.projectiles[0].at.x,1e-8);
    near(second.continuation.projectiles[0].velocity.x,referenceSpeed,1e-8);
    REQUIRE(componentHit(first,0));REQUIRE(componentHit(second,0));REQUIRE(componentHit(whole,0));
    near(requiredComponent(first,0).energyJ+requiredComponent(second,0).energyJ,requiredComponent(whole,0).energyJ,1e-6);
    CHECK(requiredComponent(first,0).hits==1);CHECK(requiredComponent(second,0).hits==0);CHECK(requiredComponent(second,0).areaM2==0);
    CHECK(componentHit(whole,1)==nullptr);near(whole.absorbedMassKg,0,1e-14);near(whole.remainingMassKg,1,1e-14);
}

TEST_CASE("liquid scene uses actual dry partial and full occupancy without density scaling", "[damage][liquid-scene]") {
    const auto frame=liquidFrame();
    for(const double fraction:{0.,.5,1.})for(const double y:{-.5,.5}) {
        INFO("fraction="<<fraction<<" y="<<y);const auto result=liquidRun(frame,liquidPoint({-1,y,0}),liquidEnvironment(.02),liquidInputs(fraction));
        REQUIRE(result.valid);REQUIRE(result.continuation.projectiles.size()==1);REQUIRE(componentHit(result,0));
        const bool wet=fraction==1||(fraction==.5&&y<0);const double speed=wet?100/1.1:100;
        near(result.continuation.projectiles[0].velocity.x,speed,1e-7);
        near(requiredComponent(result,0).energyJ,.5*(10000-speed*speed),1e-5);
        CHECK(requiredComponent(result,0).hits==1);near(result.remainingMassKg,1,1e-14);
    }
}

TEST_CASE("liquid capacity subdivision introduces neither walls nor repeated holes", "[damage][liquid-scene]") {
    const auto frame=liquidFrame();const auto field=liquidPoint();const auto environment=liquidEnvironment(.025);
    const auto reference=liquidRun(frame,field,environment,liquidInputs());REQUIRE(reference.valid);
    for(const unsigned count:{2u,127u,1024u}) {
        INFO("cells="<<count);const auto result=liquidRun(frame,field,environment,liquidInputs(1,count));
        REQUIRE(result.valid);REQUIRE(result.continuation.projectiles.size()==1);REQUIRE(componentHit(result,0));
        near(result.continuation.projectiles[0].velocity.x,reference.continuation.projectiles[0].velocity.x,1e-6);
        near(result.continuation.projectiles[0].at.x,reference.continuation.projectiles[0].at.x,1e-7);
        near(requiredComponent(result,0).energyJ,requiredComponent(reference,0).energyJ,1e-4);
        CHECK(requiredComponent(result,0).hits==1);near(requiredComponent(result,0).areaM2,.001,1e-15);
    }
    // The complete admitted cell bound exercises the same BVH/overlap path,
    // without inventing tiny positive cells for an actually empty tank.
    const auto large=liquidRun(frame,field,liquidEnvironment(.001),liquidInputs(0,65536));REQUIRE(large.valid);
    near(large.continuation.projectiles[0].velocity.x,100,1e-12);
}

TEST_CASE("tangential point and rod material is loaded once across exact same tank capacity seams", "[damage][liquid-scene][liquid-tangential-partition]") {
    const double dt=.0001,speed=100/(1+10*dt),impulse=100-speed,energy=.5*(10000-speed*speed);
    for(const bool rod:{false,true})for(const unsigned count:{1u,2u,4u,128u})for(const bool probes:{false,true})for(const bool reverse:{false,true}) {
        INFO("rod="<<rod<<" cells="<<count<<" distinct probes="<<probes<<" reverse="<<reverse);
        auto frame=liquidFrame();if(probes)frame.components.push_back(frame.components[0]);
        auto inputs=liquidInputs(1,count);auto& cells=inputs.recipients[0].cells;
        if(probes)for(std::size_t i=0;i<cells.size();++i)cells[i].componentIndex=i%2;
        const std::size_t chosen=count==1?0:static_cast<std::size_t>(count/2-1);
        const int winner=probes?static_cast<int>(chosen%2):0;
        if(reverse)std::reverse(cells.begin(),cells.end());
        auto field=liquidPoint({.5,rod?-.25:0,0},{0,0,100});auto& p=field.projectiles[0];
        if(rod){p.segment=true;p.end={.5,.25,0};p.endVelocity=p.velocity;}
        const auto result=liquidRun(frame,field,liquidEnvironment(dt),inputs);noBladderPuncture(result);
        REQUIRE(result.continuation.projectiles.size()==1);const auto& end=result.continuation.projectiles[0];
        near(end.velocity.z,speed,1e-7);if(rod)near(end.endVelocity.z,speed,1e-7);
        near(result.transferredMomentumEcef.z,impulse,1e-7);
        near(result.victims[0].impact.angularImpulseBody.y,-.5*impulse,1e-7);
        near(result.victims[0].impact.angularImpulseBody.x,0,1e-10);
        double work=0;for(const auto& hit:result.victims[0].impact.components) {
            work+=hit.energyJ;if(hit.component!=winner)CHECK(hit.energyJ==0);
        }
        near(work,energy,1e-5);REQUIRE(componentHit(result,winner));near(requiredComponent(result,winner).energyJ,energy,1e-5);
        near(length(V3{0,0,100}-result.remainingMomentumEcef-result.transferredMomentumEcef),0,1e-8);
        near(result.remainingMassKg,1,1e-14);CHECK(result.absorbedMassKg==0);near(length(result.airMomentumEcef),0,1e-14);
    }
}

TEST_CASE("exact distinct tank interface chooses entered side or stable tangential owner once", "[damage][liquid-scene][liquid-tangential-partition]") {
    const double dt=1e-7;
    for(const bool rod:{false,true})for(const bool alternateIds:{false,true})for(const bool reverse:{false,true})for(const double normal:{-50.,0.,50.}) {
        INFO("rod="<<rod<<" alternate IDs="<<alternateIds<<" reverse="<<reverse<<" normal velocity="<<normal);
        auto frame=liquidFrame();frame.components.push_back(frame.components[0]);frame.components[1].tank=1;
        auto inputs=liquidInputs(1,2);auto& recipient=inputs.recipients[0];recipient.tanks[0].actualContentsKg=2000;
        recipient.tanks.push_back({1,800,.5,1600,{{0,1,0},1}});
        recipient.cells[1].nativeTank=1;recipient.cells[1].componentIndex=1;
        if(alternateIds){recipient.cells[0].stableCellId=11;recipient.cells[1].stableCellId=3;}
        if(reverse){std::reverse(recipient.cells.begin(),recipient.cells.end());std::reverse(recipient.tanks.begin(),recipient.tanks.end());}
        const int winner=normal>0?1:(normal<0?0:(alternateIds?1:0));const double k=winner==0?.1:.2;
        const V3 velocity{normal,0,100};const double magnitude=length(velocity),ratio=1/(1+k*magnitude*dt);
        auto field=liquidPoint({.5,rod?-.25:.1,0},velocity);auto& p=field.projectiles[0];
        if(rod){p.segment=true;p.end={.5,.25,0};p.endVelocity=velocity;}
        const auto result=liquidRun(frame,field,liquidEnvironment(dt),inputs);REQUIRE(result.valid);REQUIRE(result.victims.size()==1);
        REQUIRE(result.continuation.projectiles.size()==1);const auto& end=result.continuation.projectiles[0];
        near(length(end.velocity-velocity*ratio),0,1e-8);if(rod)near(length(end.endVelocity-velocity*ratio),0,1e-8);
        const V3 reaction=velocity*(1-ratio);near(length(result.transferredMomentumEcef-reaction),0,1e-8);
        const V3 lever{.5,rod?0:.1,0};near(length(result.victims[0].impact.angularImpulseBody-cross(lever,reaction)),0,1e-8);
        const double energy=.5*magnitude*magnitude*(1-ratio)*(1+ratio);double work=0;
        for(const auto& hit:result.victims[0].impact.components){work+=hit.energyJ;if(hit.component!=winner)near(hit.energyJ,0,1e-12);}
        REQUIRE(componentHit(result,winner));near(requiredComponent(result,winner).energyJ,energy,1e-6);near(work,energy,1e-6);
        near(length(velocity-result.remainingMomentumEcef-result.transferredMomentumEcef),0,1e-9);
        near(result.remainingMassKg,1,1e-14);CHECK(result.absorbedMassKg==0);
    }
}

TEST_CASE("finite dry gap between capacity faces is never filled by rod interval ownership", "[damage][liquid-scene][liquid-tangential-partition]") {
    const auto frame=liquidFrame();const double dt=.0001,k=.5*1.2*1.3*.001,speed=100/(1+k*100*dt);
    for(const double halfGap:{.001,std::ldexp(1.,-30)})for(const bool reverse:{false,true}) {
        INFO("half gap="<<halfGap<<" reverse="<<reverse);auto inputs=liquidInputs(1,2);auto& recipient=inputs.recipients[0];
        recipient.cells[0].bodyBounds.high.x=.5-halfGap;recipient.cells[1].bodyBounds.low.x=.5+halfGap;
        recipient.tanks[0].actualContentsKg=4000-8000*halfGap;if(reverse)std::reverse(recipient.cells.begin(),recipient.cells.end());
        auto field=liquidPoint({.5,-.25,0},{0,0,100});auto& p=field.projectiles[0];p.segment=true;p.end={.5,.25,0};p.endVelocity=p.velocity;
        auto environment=liquidEnvironment(dt);environment.airDensity=1.2;
        const auto result=liquidRun(frame,field,environment,inputs);noBladderPuncture(result);REQUIRE(result.continuation.projectiles.size()==1);
        const auto& end=result.continuation.projectiles[0];near(end.velocity.z,speed,1e-8);near(end.endVelocity.z,speed,1e-8);
        near(result.airMomentumEcef.z,100-speed,1e-8);near(length(result.transferredMomentumEcef),0,1e-14);
        CHECK(result.victims[0].impact.components.empty());CHECK(result.projectilesStriking==0);CHECK_FALSE(result.firstStrikeAgeS);
        near(result.remainingMassKg,1,1e-14);CHECK(result.absorbedMassKg==0);
    }
}

TEST_CASE("tangential point changes actual tank law when stable interface ownership changes", "[damage][liquid-scene][liquid-tangential-partition]") {
    auto frame=liquidFrame();frame.components.push_back(frame.components[0]);frame.components[1].tank=1;
    const auto field=liquidPoint({.5,-.01,0},{0,100,0});const double duration=.001;
    const double transition=std::expm1(.1*.01)/(.1*100),firstSpeed=100*std::exp(-.1*.01);
    const double remaining=duration-transition,lastSpeed=firstSpeed/(1+.2*firstSpeed*remaining);
    const double lastPosition=std::log1p(.2*firstSpeed*remaining)/.2;
    const double firstWork=.5*(10000-firstSpeed*firstSpeed),lastWork=.5*(firstSpeed*firstSpeed-lastSpeed*lastSpeed);
    for(const bool reverse:{false,true})for(const double air:{0.,1.2}) {
        INFO("reverse="<<reverse<<" air density="<<air);auto inputs=liquidInputs();auto& r=inputs.recipients[0];r.tanks[0].actualContentsKg=2000;
        r.tanks.push_back({1,800,.5,1600,{{0,1,0},1}});
        r.cells={{1,0,0,0,{{0,-1,-1},{.5,0,1}}},{4,0,0,0,{{0,0,-1},{.5,1,1}}},
            {2,1,1,0,{{.5,-1,-1},{1,0,1}}},{3,1,1,0,{{.5,0,-1},{1,1,1}}}};
        if(reverse){std::reverse(r.cells.begin(),r.cells.end());std::reverse(r.tanks.begin(),r.tanks.end());}
        // Lower interface: ID1/TK0. Upper interface: ID3/TK1. Capacity is
        // continuous in each tank, but the admitted physical law changes at y0.
        auto environment=liquidEnvironment(duration);environment.airDensity=air;
        const auto whole=liquidRun(frame,field,environment,inputs);REQUIRE(whole.valid);
        environment.maxFlightTimeS=transition;
        const auto before=liquidRun(frame,field,environment,inputs);REQUIRE(before.valid);
        environment.maxFlightTimeS=remaining;
        const auto after=liquidRun(frame,before.continuation,environment,inputs);REQUIRE(after.valid);
        REQUIRE(whole.continuation.projectiles.size()==1);REQUIRE(after.continuation.projectiles.size()==1);
        const auto& end=whole.continuation.projectiles[0];const auto& split=after.continuation.projectiles[0];
        near(end.velocity.y,lastSpeed,1e-7);near(end.at.y,lastPosition,1e-9);
        near(split.velocity.y,lastSpeed,1e-7);near(split.at.y,lastPosition,1e-9);
        REQUIRE(componentHit(whole,0));REQUIRE(componentHit(whole,1));
        near(requiredComponent(whole,0).energyJ,firstWork,1e-5);near(requiredComponent(whole,1).energyJ,lastWork,1e-5);
        double splitWork=0;for(const auto& row:before.victims[0].impact.components)splitWork+=row.energyJ;
        for(const auto& row:after.victims[0].impact.components)splitWork+=row.energyJ;
        near(splitWork,firstWork+lastWork,1e-5);near(whole.transferredMomentumEcef.y,100-lastSpeed,1e-7);
        near(whole.victims[0].impact.angularImpulseBody.z,.5*(100-lastSpeed),1e-7);
        near(length(V3{0,100,0}-whole.remainingMomentumEcef-whole.transferredMomentumEcef),0,1e-8);
    }
}

TEST_CASE("liquid boundary flight remains invariant under represented Earth scale translation", "[damage][liquid-scene][liquid-earth-translation]") {
    const auto frame=liquidFrame();auto inputs=liquidInputs();inputs.recipients[0].tanks[0].dragCoefficient=1;
    const double mass=.006460604265402844,area=.0001317326963161798,duration=.0001;
    const V3 earth{4194304,-4194304,4194304},moving{220,0,0},velocity{220,0,1800};
    // Both runs receive the SAME represented local start. The reference does
    // not demand recovery of information already rounded in the ECEF input.
    const V3 represented=earth+V3{.5,0,1-1e-5}-earth;const double distance=1-represented.z;
    const double liquidK=1000*area/(2*mass),exitTime=std::expm1(liquidK*distance)/(liquidK*1800);
    const double wetSpeed=1800*std::exp(-liquidK*distance);const V3 exitVelocity{220,0,wetSpeed};
    const double airK=.660112*1.3*area/(2*mass),speed=length(exitVelocity),dryTime=duration-exitTime;
    const V3 expectedVelocity=exitVelocity*(1/(1+airK*speed*dryTime));
    const V3 expectedPosition=represented+moving*exitTime+V3{0,0,distance}+
        exitVelocity*(std::log1p(airK*speed*dryTime)/(airK*speed));
    const V3 liquidReaction{0,0,mass*(1800-wetSpeed)};
    const double liquidWork=.5*mass*(1800-wetSpeed)*(1800+wetSpeed);
    const double velocityBudget=inputs.controls.absoluteVelocityToleranceMs+inputs.controls.relativeTolerance*length(velocity);
    const double momentumBudget=mass*velocityBudget,workBudget=mass*length(velocity)*velocityBudget;
    std::optional<SceneAssessmentV1> local;
    REQUIRE(exitTime>0);REQUIRE(exitTime<duration);
    for(const V3 origin:{V3{},earth}) {
        INFO("origin="<<origin.x<<","<<origin.y<<","<<origin.z);
        auto field=liquidPoint(origin+represented,velocity);field.caseKg=mass;field.burst.at=field.projectiles[0].at;
        auto& source=field.projectiles[0];source.massKg=mass;source.areaM2=area;
        auto environment=liquidEnvironment(duration);environment.airDensity=.660112;
        const auto result=liquidRun(frame,field,environment,inputs,liquidTarget(origin,moving));REQUIRE(result.valid);
        REQUIRE(result.continuation.projectiles.size()==1);const auto& end=result.continuation.projectiles[0];
        near(length(end.velocity-expectedVelocity),0,velocityBudget);near(length(end.at-origin-expectedPosition),0,1e-7);
        near(length(result.transferredMomentumEcef-liquidReaction),0,momentumBudget);
        near(result.victims[0].impact.angularImpulseBody.y,-.5*liquidReaction.z,.5*momentumBudget);
        REQUIRE(componentHit(result,0));near(requiredComponent(result,0).energyJ,liquidWork,workBudget);
        near(length(velocity*mass-result.remainingMomentumEcef-result.transferredMomentumEcef-result.airMomentumEcef),0,1e-8);
        near(result.remainingMassKg,mass,1e-14);CHECK(result.absorbedMassKg==0);
        near(length(result.continuation.burst.at-field.burst.at),0,1e-12);
        if(!local)local=result;
        else {
            const auto& nearPoint=local->continuation.projectiles[0];
            near(length(end.velocity-nearPoint.velocity),0,1e-10);
            near(length(end.at-origin-nearPoint.at),0,1e-7);
            near(length(result.transferredMomentumEcef-local->transferredMomentumEcef),0,1e-12);
            near(length(result.victims[0].impact.angularImpulseBody-local->victims[0].impact.angularImpulseBody),0,1e-12);
            near(requiredComponent(result,0).energyJ,requiredComponent(*local,0).energyJ,1e-10);
        }
    }
}

TEST_CASE("dry point entry after positive flight age makes causal represented clock progress", "[damage][liquid-scene][liquid-clock-progress]") {
    const auto frame=liquidFrame();auto inputs=liquidInputs();inputs.recipients[0].tanks[0].dragCoefficient=1;
    const double mass=.006460604265402844,area=.0001317326963161798,duration=.15,splitTime=.04;
    const double airK=.660112*1.3*area/(2*mass),liquidK=1000*area/(2*mass);
    // A faster translating tank catches the dry air-decelerating point.
    // Solve its actual air trajectory, not a straight chord's event time.
    double low=0,high=duration;
    for(unsigned i=0;i<100;++i){const double t=low+(high-low)*.5;
        const double localX=6+std::log1p(airK*185*t)/airK-220*t;
        if(localX>1)low=t;else high=t;}
    const double entry=low+(high-low)*.5,initialRelative=220-185/(1+airK*185*entry);
    REQUIRE(entry>splitTime);REQUIRE(entry<duration);
    const double elapsed=duration-entry,relative=initialRelative/(1+liquidK*initialRelative*elapsed);
    const double expectedPosition=220*duration+1-std::log1p(liquidK*initialRelative*elapsed)/liquidK;
    auto field=liquidPoint({6,0,0},{185,0,0});field.caseKg=mass;field.burst.at=field.projectiles[0].at;
    field.projectiles[0].massKg=mass;field.projectiles[0].areaM2=area;
    auto environment=liquidEnvironment(duration);environment.airDensity=.660112;
    const auto whole=liquidRun(frame,field,environment,inputs,liquidTarget({},{220,0,0}));REQUIRE(whole.valid);
    environment.maxFlightTimeS=splitTime;
    const auto first=liquidRun(frame,field,environment,inputs,liquidTarget({},{220,0,0}));REQUIRE(first.valid);
    environment.maxFlightTimeS=duration-splitTime;
    const auto second=liquidRun(frame,first.continuation,environment,inputs,liquidTarget({220*splitTime,0,0},{220,0,0}));REQUIRE(second.valid);
    REQUIRE(whole.continuation.projectiles.size()==1);REQUIRE(second.continuation.projectiles.size()==1);
    const auto& end=whole.continuation.projectiles[0];const auto& resumed=second.continuation.projectiles[0];
    const double budget=inputs.controls.absoluteVelocityToleranceMs+inputs.controls.relativeTolerance*220;
    near(end.velocity.x,220-relative,budget);near(end.at.x,expectedPosition,1e-6);
    near(length(end.velocity-resumed.velocity),0,2*budget);near(length(end.at-resumed.at),0,2e-6);
    REQUIRE(componentHit(whole,0));CHECK(requiredComponent(whole,0).energyJ>0);
    near(length(V3{185*mass,0,0}-whole.remainingMomentumEcef-whole.transferredMomentumEcef-whole.airMomentumEcef),0,1e-8);
    REQUIRE(whole.firstStrikeAgeS);near(*whole.firstStrikeAgeS,entry,1e-7);
}

TEST_CASE("oblique moving liquid face admits a later air decelerated point without event cycling", "[damage][liquid-scene][liquid-clock-progress]") {
    const auto frame=liquidFrame();const auto inputs=liquidInputs();const double duration=.1,k=.5*1.2*1.3*.001;
    double low=0,high=duration;
    for(unsigned i=0;i<100;++i){const double t=low+(high-low)*.5;
        if(1.5+std::log1p(k*100*t)/k-110*t>1)low=t;else high=t;}
    const double entry=low+(high-low)*.5,initialRelative=110-100/(1+k*100*entry);
    const double relative=initialRelative/(1+.1*initialRelative*(duration-entry));
    for(const double angle:{0.,37.}) {
        INFO("body orientation degrees="<<angle);const double radians=angle*std::acos(-1.)/180;
        const V3 forward{std::cos(radians),std::sin(radians),0},right{-forward.y,forward.x,0};
        auto field=liquidPoint(forward*1.5,forward*100);field.burst.at=field.projectiles[0].at;
        auto target=liquidTarget({},forward*110);target.bodyToEcef={forward,right,V3{0,0,1}};
        auto environment=liquidEnvironment(duration);environment.airDensity=1.2;
        const auto result=liquidRun(frame,field,environment,inputs,target);REQUIRE(result.valid);
        REQUIRE(result.continuation.projectiles.size()==1);const auto& end=result.continuation.projectiles[0];
        near(length(end.velocity-forward*(110-relative)),0,inputs.controls.absoluteVelocityToleranceMs+inputs.controls.relativeTolerance*110);
        REQUIRE(result.firstStrikeAgeS);near(*result.firstStrikeAgeS,entry,1e-7);
        REQUIRE(componentHit(result,0));CHECK(requiredComponent(result,0).energyJ>0);
        near(length(forward*100-result.remainingMomentumEcef-result.transferredMomentumEcef-result.airMomentumEcef),0,1e-8);
    }
}

TEST_CASE("stiff point liquid entry resolves its material boundary before smooth integration", "[damage][liquid-scene][liquid-stiff-entry]") {
    const auto frame=liquidFrame();auto inputs=liquidInputs();inputs.recipients[0].tanks[0].dragCoefficient=1;
    const double mass=.001,area=.0001,duration=.00001,airK=.66*1.3*area/(2*mass),liquidK=1000*area/(2*mass);
    const double entry=std::expm1(airK*.01)/(airK*2000),speed=2000*std::exp(-airK*.01);
    const double remaining=duration-entry,lastSpeed=speed/(1+liquidK*speed*remaining),position=std::log1p(liquidK*speed*remaining)/liquidK;
    REQUIRE(entry>0);REQUIRE(entry<duration);
    auto field=liquidPoint({-.01,0,0},{2000,0,0});field.caseKg=mass;field.burst.at=field.projectiles[0].at;
    field.projectiles[0].massKg=mass;field.projectiles[0].areaM2=area;
    auto environment=liquidEnvironment(duration);environment.airDensity=.66;
    const auto result=liquidRun(frame,field,environment,inputs);REQUIRE(result.valid);
    REQUIRE(result.continuation.projectiles.size()==1);const auto& end=result.continuation.projectiles[0];
    const double budget=inputs.controls.absoluteVelocityToleranceMs+inputs.controls.relativeTolerance*2000;
    near(end.velocity.x,lastSpeed,budget);near(end.at.x,position,inputs.controls.absolutePositionToleranceM);
    REQUIRE(componentHit(result,0));near(requiredComponent(result,0).energyJ,.5*mass*(speed-lastSpeed)*(speed+lastSpeed),mass*2000*budget);
    near(result.transferredMomentumEcef.x,mass*(speed-lastSpeed),mass*budget);
    near(length(V3{mass*2000,0,0}-result.remainingMomentumEcef-result.transferredMomentumEcef-result.airMomentumEcef),0,1e-8);
    REQUIRE(result.firstStrikeAgeS);near(*result.firstStrikeAgeS,entry,1e-8);
}

TEST_CASE("tangent point accelerated out of a liquid face receives no future liquid work", "[damage][liquid-scene][liquid-zero-time-phase]") {
    const auto frame=liquidFrame();const double dt=.0001;const auto field=liquidPoint({.5,0,-1},{100,0,0});
    auto environment=liquidEnvironment(dt);environment.gravityEcef={0,0,-9.80665};
    const auto result=liquidRun(frame,field,environment,liquidInputs());REQUIRE(result.valid);
    REQUIRE(result.continuation.projectiles.size()==1);const auto& end=result.continuation.projectiles[0];
    near(end.velocity.x,100,1e-10);near(end.velocity.z,-9.80665*dt,1e-10);
    near(end.at.x,.5+100*dt,1e-10);near(end.at.z,-1-.5*9.80665*dt*dt,1e-10);
    near(length(result.transferredMomentumEcef),0,1e-12);near(length(result.airMomentumEcef),0,1e-12);
    for(const auto& row:result.victims[0].impact.components)CHECK(row.energyJ==0);
    near(length(V3{100,0,0}+result.gravityImpulseEcef-result.remainingMomentumEcef),0,1e-10);
}

TEST_CASE("immediate tangent departure preserves a real dry gap before a later tank", "[damage][liquid-scene][liquid-right-limit-gap]") {
    auto frame=liquidFrame();frame.components.push_back(frame.components[0]);frame.components[1].tank=1;
    const double gap=1e-8,duration=.0001,entry=std::sqrt(2*gap/10);
    const auto field=liquidPoint({.5,0,0},{100,0,0});
    for(const double sign:{-1.,1.})for(const bool reverse:{false,true}) {
        INFO("acceleration sign="<<sign<<" reverse="<<reverse);auto inputs=liquidInputs();auto& r=inputs.recipients[0];
        r.tanks[0].actualContentsKg=2000;r.tanks.push_back({1,1000,.2,2000*(1-gap),{{0,1,0},1}});
        r.cells={{1,0,0,0,{{0,-1,0},{1,1,1}}},{2,1,1,0,{{0,-1,-1},{1,1,-gap}}}};
        if(sign>0){r.cells[0].bodyBounds.low.z=-1;r.cells[0].bodyBounds.high.z=0;
            r.cells[1].bodyBounds.low.z=gap;r.cells[1].bodyBounds.high.z=1;}
        if(reverse){std::reverse(r.cells.begin(),r.cells.end());std::reverse(r.tanks.begin(),r.tanks.end());}
        auto environment=liquidEnvironment(duration);environment.gravityEcef={0,0,sign*10};
        const auto whole=liquidRun(frame,field,environment,inputs);REQUIRE(whole.valid);
        REQUIRE(whole.continuation.projectiles.size()==1);
        if(const auto* a=componentHit(whole,0))CHECK(a->energyJ==0);
        const auto* b=componentHit(whole,1);CHECK(b);if(b)CHECK(b->energyJ>0);
        environment.maxFlightTimeS=entry*.99;
        const auto dry=liquidRun(frame,field,environment,inputs);REQUIRE(dry.valid);
        for(const auto& hit:dry.victims[0].impact.components)CHECK(hit.energyJ==0);
        near(length(dry.transferredMomentumEcef),0,1e-12);
        REQUIRE(dry.continuation.projectiles.size()==1);near(dry.continuation.projectiles[0].velocity.x,100,1e-10);
        near(dry.continuation.projectiles[0].at.z,sign*5*environment.maxFlightTimeS*environment.maxFlightTimeS,1e-12);
        environment.maxFlightTimeS=entry;
        const auto first=liquidRun(frame,field,environment,inputs);REQUIRE(first.valid);
        for(const auto& hit:first.victims[0].impact.components)near(hit.energyJ,0,1e-10);
        environment.maxFlightTimeS=duration-entry;
        const auto second=liquidRun(frame,first.continuation,environment,inputs);REQUIRE(second.valid);
        REQUIRE(second.continuation.projectiles.size()==1);
        near(length(whole.continuation.projectiles[0].velocity-second.continuation.projectiles[0].velocity),0,1e-7);
        near(length(whole.continuation.projectiles[0].at-second.continuation.projectiles[0].at),0,1e-9);
        if(const auto* a=componentHit(second,0))CHECK(a->energyJ==0);
        const auto* later=componentHit(second,1);CHECK(later);if(b&&later)near(b->energyJ,later->energyJ,1e-6);
        near(length(V3{100,0,0}+whole.gravityImpulseEcef-whole.transferredMomentumEcef-whole.airMomentumEcef-whole.remainingMomentumEcef),0,1e-8);
    }
}

TEST_CASE("first liquid interaction is earliest across projectile storage order", "[damage][liquid-scene][liquid-first-interaction]") {
    const auto frame=liquidFrame();auto field=liquidPoint();const auto interior=liquidPoint({.25,0,0});
    field.projectiles.push_back(interior.projectiles[0]);field.caseKg=2;
    for(const bool reverse:{false,true}) {
        INFO("reverse="<<reverse);if(reverse)std::reverse(field.projectiles.begin(),field.projectiles.end());
        const auto result=liquidRun(frame,field,liquidEnvironment(.015),liquidInputs());REQUIRE(result.valid);
        REQUIRE(result.firstStrikeAgeS);CHECK(*result.firstStrikeAgeS==0);CHECK(result.projectilesStriking==2);
    }
}

TEST_CASE("one native liquid tank retains actual per-probe work without duplicate punctures", "[damage][liquid-scene]") {
    auto frame=liquidFrame();frame.components.push_back(frame.components[0]);auto inputs=liquidInputs(1,2);
    inputs.recipients[0].cells[1].componentIndex=1;
    const auto result=liquidRun(frame,liquidPoint(),liquidEnvironment(.025),inputs);REQUIRE(result.valid);
    REQUIRE(componentHit(result,0));REQUIRE(componentHit(result,1));
    near(requiredComponent(result,0).energyJ,5000*(1-std::exp(-.1)),1e-5);
    near(requiredComponent(result,1).energyJ,5000*(std::exp(-.1)-std::exp(-.2)),1e-5);
    CHECK(requiredComponent(result,0).hits==1);CHECK(requiredComponent(result,1).hits==0);
    near(requiredComponent(result,0).areaM2,.001,1e-15);CHECK(requiredComponent(result,1).areaM2==0);
}

TEST_CASE("liquid capacity union preserves a real dry gap between two occupied slabs", "[damage][liquid-scene]") {
    const auto frame=liquidFrame();auto inputs=liquidInputs(1,2);auto& recipient=inputs.recipients[0];
    recipient.cells[0].bodyBounds.high.x=.4;recipient.cells[1].bodyBounds.low.x=.6;
    recipient.tanks[0].actualContentsKg=3200; // 0.8 m total wet chord, physical density unchanged.
    const double k=.1,firstSpeed=100*std::exp(-k*.4),lastSpeed=100*std::exp(-k*.8),duration=.025;
    const double exitTime=.01+std::expm1(k*.4)/(k*100)+.2/firstSpeed+std::expm1(k*.4)/(k*firstSpeed);
    const auto result=liquidRun(frame,liquidPoint(),liquidEnvironment(duration),inputs);REQUIRE(result.valid);
    REQUIRE(result.continuation.projectiles.size()==1);REQUIRE(componentHit(result,0));const auto& p=result.continuation.projectiles[0];
    near(p.velocity.x,lastSpeed,1e-7);near(p.at.x,1+lastSpeed*(duration-exitTime),1e-7);
    near(requiredComponent(result,0).energyJ,.5*(10000-lastSpeed*lastSpeed),1e-5);
    near(result.transferredMomentumEcef.x,100-lastSpeed,1e-7);CHECK(requiredComponent(result,0).hits==1);
    near(result.remainingMassKg,1,1e-14);CHECK(result.absorbedMassKg==0);
}

TEST_CASE("liquid impulse energy and moving-origin torque retain a common velocity boost", "[damage][liquid-scene]") {
    const auto frame=liquidFrame();const auto inputs=liquidInputs();const auto environment=liquidEnvironment(.02);
    for(const V3 boost:{V3{},V3{50,20,-30}}) {
        const V3 origin={10,4,2};const auto field=liquidPoint(origin+V3{-1,.2,0},boost+V3{100,0,0});
        const auto result=liquidRun(frame,field,environment,inputs,liquidTarget(origin,boost));REQUIRE(result.valid);
        REQUIRE(result.continuation.projectiles.size()==1);REQUIRE(componentHit(result,0));
        const double speed=100/1.1,impulse=100-speed;const auto& p=result.continuation.projectiles[0];
        near(p.velocity.x,boost.x+speed,1e-7);near(result.transferredMomentumEcef.x,impulse,1e-7);
        near(result.victims[0].impact.angularImpulseBody.z,-.2*impulse,1e-7);
        const double initialEnergy=.5*dot(field.projectiles[0].velocity,field.projectiles[0].velocity),finalEnergy=.5*dot(p.velocity,p.velocity);
        near(initialEnergy-finalEnergy,requiredComponent(result,0).energyJ+dot(boost,result.transferredMomentumEcef),1e-5);
        near(length(field.projectiles[0].velocity-result.remainingMomentumEcef-result.transferredMomentumEcef),0,1e-7);
    }
}

TEST_CASE("liquid scene Cd half and double screening preserves finite chord and boosted balances", "[damage][liquid-scene][liquid-cd-sensitivity]") {
    const auto frame=liquidFrame();const V3 moving{20,-5,0};const auto field=liquidPoint({-1,.2,0},moving+V3{100,0,0});
    const double duration=.04;double previousResidual=100,previousWork=0;
    for(const double cd:{.5,1.,2.}) {
        INFO("explicit liquid Cd="<<cd);auto inputs=liquidInputs();inputs.recipients[0].tanks[0].dragCoefficient=cd;
        const double k=.5*cd,residual=100*std::exp(-k),residence=std::expm1(k)/(k*100);
        REQUIRE(.01+residence<duration); // Fixed window includes the complete physical chord for all three inputs.
        const auto result=liquidRun(frame,field,liquidEnvironment(duration),inputs,liquidTarget({},moving));REQUIRE(result.valid);
        REQUIRE(result.continuation.projectiles.size()==1);REQUIRE(componentHit(result,0));const auto& p=result.continuation.projectiles[0];
        near(p.velocity.x,moving.x+residual,1e-7);
        near(p.at.x,moving.x*duration+1+residual*(duration-.01-residence),1e-7);
        near(requiredComponent(result,0).energyJ,.5*(10000-residual*residual),1e-5);
        near(result.transferredMomentumEcef.x,100-residual,1e-7);
        near(result.victims[0].impact.angularImpulseBody.z,-.2*(100-residual),1e-7);
        near(length(field.projectiles[0].velocity-result.remainingMomentumEcef-result.transferredMomentumEcef),0,1e-7);
        const double loss=.5*(dot(field.projectiles[0].velocity,field.projectiles[0].velocity)-dot(p.velocity,p.velocity));
        near(loss,requiredComponent(result,0).energyJ+dot(moving,result.transferredMomentumEcef),1e-5);
        near(result.remainingMassKg,1,1e-14);CHECK(result.absorbedMassKg==0);CHECK(requiredComponent(result,0).hits==1);
        CHECK(residual<previousResidual);CHECK(requiredComponent(result,0).energyJ>previousWork);
        previousResidual=residual;previousWork=requiredComponent(result,0).energyJ;
    }
}

TEST_CASE("liquid scene Cd half and double screening preserves partial rod mass moment and energy", "[damage][liquid-scene][liquid-cd-sensitivity]") {
    const auto frame=liquidFrame();auto field=liquidPoint({.25,-.5,0});auto& p=field.projectiles[0];
    p.segment=true;p.end={.25,.5,0};p.endVelocity=p.velocity;const double dt=1e-8;
    for(const double cd:{.5,1.,2.}) {
        INFO("explicit liquid Cd="<<cd);auto inputs=liquidInputs(.5);inputs.recipients[0].tanks[0].dragCoefficient=cd;
        const auto result=liquidRun(frame,field,liquidEnvironment(dt),inputs);REQUIRE(result.valid);
        REQUIRE(result.continuation.projectiles.size()==1);REQUIRE(componentHit(result,0));const auto& out=result.continuation.projectiles[0];
        // Prescribed instantaneous shape-force integrals give endpoint
        // accelerations (-6250,+1250)*Cd and reaction force 2500*Cd N.
        // The fixed short interval bounds the Taylor remainder independently
        // of any engagement outcome; the physical law is never re-fit.
        near(out.velocity.x,100-6250*cd*dt,2e-8);near(out.endVelocity.x,100+1250*cd*dt,2e-8);
        near(result.transferredMomentumEcef.x,2500*cd*dt,2e-9);
        near(result.victims[0].impact.angularImpulseBody.z,625*cd*dt,2e-9);
        near(requiredComponent(result,0).energyJ,250000*cd*dt,2e-7);
        const V3 mean=(out.velocity+out.endVelocity)*.5,difference=out.endVelocity-out.velocity;
        near(.5*dot(mean,mean)+dot(difference,difference)/24+requiredComponent(result,0).energyJ,5000,2e-6);
        near(length(p.velocity-result.remainingMomentumEcef-result.transferredMomentumEcef),0,1e-8);
        // The complete source rod is initially inside capacity; drag is real,
        // but no bladder surface has yet been crossed.
        near(result.remainingMassKg,1,1e-14);CHECK(result.absorbedMassKg==0);CHECK(requiredComponent(result,0).hits==0);
    }
}

TEST_CASE("partially immersed rod receives distributed consistent-mass loads", "[damage][liquid-scene]") {
    const auto frame=liquidFrame();auto field=liquidPoint({.25,-.5,0});auto& p=field.projectiles[0];
    p.segment=true;p.end={.25,.5,0};p.endVelocity=p.velocity;
    const double dt=1e-7;const auto result=liquidRun(frame,field,liquidEnvironment(dt),liquidInputs(.5));REQUIRE(result.valid);
    REQUIRE(result.continuation.projectiles.size()==1);const auto& end=result.continuation.projectiles[0];
    // Uniform force density -1000 N, immersed s=[0,1/2]: F0=-375,
    // F1=-125 N. Inverting m/6 [[2,1],[1,2]] gives (-1250,+250).
    near(end.velocity.x,100-1250*dt,2e-8);near(end.endVelocity.x,100+250*dt,2e-8);
    near(result.transferredMomentumEcef.x,500*dt,2e-9);
    near(result.victims[0].impact.angularImpulseBody.z,125*dt,2e-9);
    REQUIRE(componentHit(result,0));near(requiredComponent(result,0).energyJ,50000*dt,2e-7);
    const V3 mean=(end.velocity+end.endVelocity)*.5,difference=end.endVelocity-end.velocity;
    near(.5*dot(mean,mean)+dot(difference,difference)/24+requiredComponent(result,0).energyJ,5000,2e-6);
    near(result.remainingMassKg,1,1e-14);CHECK(result.absorbedMassKg==0);
}

TEST_CASE("liquid flight accounts air and gravity exactly once in the momentum ledger", "[damage][liquid-scene]") {
    const auto frame=liquidFrame();const auto field=liquidPoint({.25,.2,0});
    auto environment=liquidEnvironment(.0001);environment.airDensity=1.2;environment.airVelocity={5,-3,0};environment.gravityEcef={0,0,9.81};
    const auto result=liquidRun(frame,field,environment,liquidInputs());REQUIRE(result.valid);
    const V3 initial=field.projectiles[0].velocity;
    near(length(initial+result.gravityImpulseEcef-result.transferredMomentumEcef-result.airMomentumEcef-result.remainingMomentumEcef),0,1e-8);
    near(result.gravityImpulseEcef.z,9.81*.0001,1e-14);
    // This point stays completely immersed: gravity and liquid reactions
    // remain in the ledger, while no ambient air occupies its material.
    near(length(result.airMomentumEcef),0,1e-14);CHECK(length(result.transferredMomentumEcef)>0);
}

TEST_CASE("fully immersed point and rod exclude ambient air over the same material", "[damage][liquid-scene][liquid-air-complement]") {
    const auto frame=liquidFrame();const double duration=.0001,k=.1;
    const double speed=100/(1+k*100*duration),position=.25+std::log1p(k*100*duration)/k;
    for(const bool rod:{false,true})for(const double density:{0.,1.2,2.4}) {
        INFO("rod="<<rod<<" ambient density="<<density);auto field=liquidPoint({.25,-.5,0});auto& source=field.projectiles[0];
        if(rod){source.segment=true;source.end={.25,.5,0};source.endVelocity=source.velocity;}
        auto environment=liquidEnvironment(duration);environment.airDensity=density;environment.airVelocity={5,-3,2};
        const auto result=liquidRun(frame,field,environment,liquidInputs());noBladderPuncture(result);
        REQUIRE(result.continuation.projectiles.size()==1);const auto& p=result.continuation.projectiles[0];
        // This finite interval remains strictly inside the full tank. Its
        // closed homogeneous-liquid solution contains no ambient-air term.
        near(p.at.x,position,1e-9);near(p.at.y,-.5,1e-10);near(p.at.z,0,1e-10);
        near(p.velocity.x,speed,1e-7);near(p.velocity.y,0,1e-9);near(p.velocity.z,0,1e-9);
        if(rod){near(p.end.x,position,1e-9);near(p.end.y,.5,1e-10);near(p.end.z,0,1e-10);
            near(p.endVelocity.x,speed,1e-7);near(p.endVelocity.y,0,1e-9);near(p.endVelocity.z,0,1e-9);}
        near(length(result.airMomentumEcef),0,1e-12);near(result.transferredMomentumEcef.x,100-speed,1e-7);
        REQUIRE(componentHit(result,0));near(requiredComponent(result,0).energyJ,.5*(10000-speed*speed),1e-5);
        near(length(source.velocity-result.remainingMomentumEcef-result.transferredMomentumEcef),0,1e-8);
        near(result.remainingMassKg,1,1e-14);CHECK(result.absorbedMassKg==0);
        CHECK(result.projectilesStriking==1);REQUIRE(result.firstStrikeAgeS);CHECK(*result.firstStrikeAgeS==0);
    }
}

TEST_CASE("half immersed varying velocity rod applies air only to its dry material interval", "[damage][liquid-scene][liquid-air-complement]") {
    const auto frame=liquidFrame();auto field=liquidPoint({.25,-.5,0},{50,0,0});auto& source=field.projectiles[0];
    source.segment=true;source.end={.25,.5,0};source.endVelocity={150,0,0};
    const double dt=1e-8,liquidCoefficient=.5*1000*.2*.001,airCoefficient=.5*1.2*1.3*.001;
    auto environment=liquidEnvironment(dt);environment.airDensity=1.2;
    const auto result=liquidRun(frame,field,environment,liquidInputs(.5));noBladderPuncture(result);
    REQUIRE(result.continuation.projectiles.size()==1);const auto& p=result.continuation.projectiles[0];
    // Initial material velocity is v(s)=50+100s, wet s=[0,1/2],
    // dry s=[1/2,1]. These explicit antiderivatives integrate v^2,
    // s*v^2 and v^3 independently of the production quadrature.
    const auto square=[](double s){return 2500*s+5000*s*s+(10000./3)*s*s*s;};
    const auto weighted=[](double s){return 1250*s*s+(10000./3)*s*s*s+2500*s*s*s*s;};
    const auto cube=[](double s){return 125000*s+375000*s*s+500000*s*s*s+250000*s*s*s*s;};
    const double wetSquare=square(.5),wetWeighted=weighted(.5),drySquare=square(1)-wetSquare,dryWeighted=weighted(1)-wetWeighted;
    const double force0=-liquidCoefficient*(wetSquare-wetWeighted)-airCoefficient*(drySquare-dryWeighted);
    const double force1=-liquidCoefficient*wetWeighted-airCoefficient*dryWeighted;
    // Invert the complete consistent rod mass matrix, not a wet mass or
    // a scalar fraction of whole-rod air force. O(dt^2) is <1e-9 here.
    near(p.velocity.x,50+(4*force0-2*force1)*dt,2e-9);
    near(p.endVelocity.x,150+(4*force1-2*force0)*dt,2e-9);
    near(result.airMomentumEcef.x,airCoefficient*drySquare*dt,2e-11);
    near(result.transferredMomentumEcef.x,liquidCoefficient*wetSquare*dt,2e-11);
    near(result.victims[0].impact.angularImpulseBody.z,liquidCoefficient*(.5*wetSquare-wetWeighted)*dt,2e-11);
    REQUIRE(componentHit(result,0));near(requiredComponent(result,0).energyJ,liquidCoefficient*cube(.5)*dt,2e-8);
    near(length(V3{100,0,0}-result.remainingMomentumEcef-result.transferredMomentumEcef-result.airMomentumEcef),0,1e-10);
    near(result.remainingMassKg,1,1e-14);CHECK(result.absorbedMassKg==0);
}

TEST_CASE("immersed material comoving with liquid excludes air without manufacturing liquid work", "[damage][liquid-scene][liquid-air-complement]") {
    const auto frame=liquidFrame();const V3 boost{10,20,-5};const double dt=.0001;
    for(const bool rod:{false,true}) {
        INFO("rod="<<rod);auto field=liquidPoint({.25,-.5,0},boost);auto& source=field.projectiles[0];
        if(rod){source.segment=true;source.end={.25,.5,0};source.endVelocity=boost;}
        auto environment=liquidEnvironment(dt);environment.airDensity=1.2;environment.airVelocity={-40,5,3};
        const auto result=liquidRun(frame,field,environment,liquidInputs(),liquidTarget({},boost));noBladderPuncture(result);
        REQUIRE(result.continuation.projectiles.size()==1);const auto& p=result.continuation.projectiles[0];
        near(length(p.velocity-boost),0,1e-12);near(length(p.at-source.at-boost*dt),0,1e-12);
        if(rod){near(length(p.endVelocity-boost),0,1e-12);near(length(p.end-source.end-boost*dt),0,1e-12);}
        near(length(result.airMomentumEcef),0,1e-14);near(length(result.transferredMomentumEcef),0,1e-14);
        // Zero-valued per-component bookkeeping is allowed; it must not
        // become drag work, a puncture or an actual material encounter.
        for(const auto& hit:result.victims[0].impact.components)CHECK(hit.energyJ==0);
        CHECK(result.projectilesStriking==0);CHECK_FALSE(result.firstStrikeAgeS);
        near(result.remainingMassKg,1,1e-14);CHECK(result.absorbedMassKg==0);
    }
}

TEST_CASE("dry and zero current liquid occupancy retain complete ambient air drag", "[damage][liquid-scene][liquid-air-complement]") {
    const auto frame=liquidFrame();const double dt=.0001,k=.5*1.2*1.3*.001,relative=90;
    const double speed=10+relative/(1+k*relative*dt),position=.25+10*dt+std::log1p(k*relative*dt)/k;
    for(const bool rod:{false,true})for(const double contents:{0.,.5}) {
        INFO("rod="<<rod<<" contents fraction="<<contents);
        const double y=contents==0?-.5:.6;auto field=liquidPoint({.25,y,0});auto& source=field.projectiles[0];
        if(rod){source.segment=true;source.end={.25,.9,0};source.endVelocity=source.velocity;}
        auto environment=liquidEnvironment(dt);environment.airDensity=1.2;environment.airVelocity={10,0,0};
        const auto result=liquidRun(frame,field,environment,liquidInputs(contents));noBladderPuncture(result);
        REQUIRE(result.continuation.projectiles.size()==1);const auto& p=result.continuation.projectiles[0];
        near(p.velocity.x,speed,1e-8);near(p.at.x,position,1e-9);
        if(rod){near(p.endVelocity.x,speed,1e-8);near(p.end.x,position,1e-9);}
        near(result.airMomentumEcef.x,100-speed,1e-8);near(length(result.transferredMomentumEcef),0,1e-14);
        CHECK(result.victims[0].impact.components.empty());CHECK(result.projectilesStriking==0);CHECK_FALSE(result.firstStrikeAgeS);
        near(result.remainingMassKg,1,1e-14);CHECK(result.absorbedMassKg==0);
    }
}

TEST_CASE("liquid residual still pays downstream structural skin work", "[damage][liquid-scene]") {
    auto frame=liquidFrame(true);frame.sections[0].skinAreaM2=1;frame.sections[0].skinKg=.002*2780;
    Airframe::Panel skin;skin.section=0;skin.corners={V3{1.5,-1,-1},V3{1.5,1,-1},V3{1.5,1,1},V3{1.5,-1,1}};skin.areaM2=4;
    frame.panels.push_back(skin);const auto field=liquidPoint();const double wetResidual=100*std::exp(-.1);
    const double afterSkin=thorResidual(.002,.001,1,wetResidual,0);
    const auto result=liquidRun(frame,field,liquidEnvironment(.2),liquidInputs());REQUIRE(result.valid);
    REQUIRE(result.victims[0].impact.panels.size()==1);CHECK(result.victims[0].impact.panels[0].hits==1);
    REQUIRE(componentHit(result,0));near(requiredComponent(result,0).energyJ,.5*(10000-wetResidual*wetResidual),1e-5);
    if(afterSkin>0){REQUIRE(componentHit(result,1));near(requiredComponent(result,1).energyJ,.5*afterSkin*afterSkin,1e-5);}
    else CHECK(componentHit(result,1)==nullptr);
    near(result.transferredMomentumEcef.x,100,1e-6);near(result.absorbedMassKg,1,1e-14);
}

TEST_CASE("liquid scene rejects stale incomplete overlapping and inconsistent material atomically", "[damage][liquid-scene]") {
    const auto frame=liquidFrame();const auto field=liquidPoint();const auto environment=liquidEnvironment();
    for(unsigned fault=0;fault<8;++fault) {
        INFO("fault="<<fault);auto input=liquidInputs();auto& r=input.recipients[0];
        if(fault==0)++r.generation;
        if(fault==1)r.tanks.clear();
        if(fault==2)r.tanks[0].dragCoefficient=0;
        if(fault==3)r.cells.clear();
        if(fault==4)r.tanks[0].actualContentsKg=2000;
        if(fault==5){auto copy=r.cells[0];copy.stableCellId=2;r.cells.push_back(copy);}
        if(fault==6)r.cells[0].section=8;
        if(fault==7)input.controls.maxSteps=0;
        const auto result=liquidRun(frame,field,environment,input);CHECK_FALSE(result.valid);CHECK(result.victims.empty());CHECK(result.continuation.projectiles.empty());
        CHECK(field.projectiles[0].at.x==-1);CHECK(field.projectiles[0].velocity.x==100);CHECK(field.projectiles[0].crossedFaces.empty());
    }
}

TEST_CASE("finite disjoint liquid cells cannot overflow the aggregate volume ledger", "[damage][liquid-scene][liquid-volume-overflow]") {
    const auto frame=liquidFrame();const auto field=liquidPoint();auto input=liquidInputs();
    auto& recipient=input.recipients[0];
    recipient.tanks[0].actualContentsKg=1;
    recipient.tanks[0].freeSurfaceBody={{0,1,0},1e103};
    recipient.cells={{1,0,0,0,{{0,0,0},{4.5e102,4.5e102,4.5e102}}},
                     {2,0,0,0,{{6e102,0,0},{10.5e102,4.5e102,4.5e102}}}};
    for(const auto& cell:recipient.cells) {
        const auto capacity=fluid::detail::box(cell.bodyBounds);REQUIRE(capacity);
        CHECK(std::isfinite(capacity->volume));CHECK(capacity->volume>9e307);
        const auto wet=fluid::immersedVolume(cell.bodyBounds,recipient.tanks[0].freeSurfaceBody);
        REQUIRE(wet);CHECK(std::isfinite(*wet));CHECK(*wet>9e307);
    }
    CHECK(recipient.cells[0].bodyBounds.high.x<recipient.cells[1].bodyBounds.low.x);
    const auto result=liquidRun(frame,field,liquidEnvironment(1e-8),input);
    CHECK_FALSE(result.valid);CHECK(result.victims.empty());CHECK(result.continuation.projectiles.empty());
    CHECK(field.projectiles[0].at.x==-1);CHECK(field.projectiles[0].velocity.x==100);
    CHECK(field.projectiles[0].crossedFaces.empty());
}

TEST_CASE("finite liquid tolerance inputs cannot overflow the admission budget", "[damage][liquid-scene][liquid-volume-overflow]") {
    const auto frame=liquidFrame();const auto field=liquidPoint();auto input=liquidInputs();
    input.recipients[0].tanks[0].actualContentsKg=2000;
    input.controls.absoluteVolumeToleranceM3=std::numeric_limits<double>::max();
    input.controls.relativeVolumeTolerance=std::numeric_limits<double>::max();
    const auto result=liquidRun(frame,field,liquidEnvironment(1e-8),input);
    CHECK_FALSE(result.valid);CHECK(result.victims.empty());CHECK(result.continuation.projectiles.empty());
    CHECK(field.projectiles[0].at.x==-1);CHECK(field.projectiles[0].velocity.x==100);
    CHECK(field.projectiles[0].crossedFaces.empty());
}

TEST_CASE("positive liquid contents require capacity cells even within numeric volume tolerance", "[damage][liquid-scene][liquid-volume-overflow]") {
    const auto frame=liquidFrame();const auto field=liquidPoint();auto input=liquidInputs();
    auto& recipient=input.recipients[0];recipient.cells.clear();recipient.tanks[0].actualContentsKg=1e-15;
    REQUIRE(recipient.tanks[0].actualContentsKg>0);
    REQUIRE(recipient.tanks[0].actualContentsKg/recipient.tanks[0].densityKgM3<input.controls.absoluteVolumeToleranceM3);
    const auto result=liquidRun(frame,field,liquidEnvironment(1e-8),input);
    CHECK_FALSE(result.valid);CHECK(result.victims.empty());CHECK(result.continuation.projectiles.empty());
    CHECK(field.projectiles[0].at.x==-1);CHECK(field.projectiles[0].velocity.x==100);
    CHECK(field.projectiles[0].crossedFaces.empty());
}

TEST_CASE("liquid opt-in preserves old assessment and suppresses dry orphan fuel probes", "[damage][liquid-scene]") {
    const auto frame=liquidFrame();const auto field=liquidPoint();const auto environment=liquidEnvironment();
    const std::vector<SceneVictimV1> victims={{7,AttachedGeometryView(frame),liquidTarget(),3}};
    const auto old=assessSceneV1(field,victims,environment),unlisted=assessSceneLiquidsV1(field,victims,environment,{});
    REQUIRE(old.valid);REQUIRE(unlisted.valid);near(old.absorbedMassKg,1,1e-14);near(unlisted.absorbedMassKg,old.absorbedMassKg,0);
    REQUIRE(componentHit(old,0));REQUIRE(componentHit(unlisted,0));near(requiredComponent(unlisted,0).energyJ,requiredComponent(old,0).energyJ,0);
    auto orphan=liquidInputs(0);orphan.recipients[0].cells.clear();const auto dry=liquidRun(frame,field,environment,orphan);
    REQUIRE(dry.valid);CHECK(componentHit(dry,0)==nullptr);near(dry.remainingMassKg,1,1e-14);CHECK(dry.absorbedMassKg==0);
}
