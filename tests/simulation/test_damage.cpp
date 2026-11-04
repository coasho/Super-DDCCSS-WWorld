// MS-7g's damage part (src/simulation/mission_systems/damage): the warhead each card and the rule make, the blast against an independent fit,
// THOR's penetration, every aircraft's damage data read, and a burst beside an F-16C - where the fragments go,
// what they strike, deterministic from the damage stream.
#include "simulation/mission_systems/damage/Damage.h"
#include "simulation/mission_systems/damage/State.h"
#include "simulation/mission_systems/weapon/MaterialPoint.h"
#include "mode_flights.h"
#include "simulation/flight/JsbsimModel.h"
#include "simulation/mission_systems/components/registration/BuiltInProvidersV1.h"
#include "simulation/mission_systems/components/interfaces/WarheadAdapter.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using namespace fsim::ms;
using namespace fsim::ms::damage;

namespace {
Target sceneTarget(V3 at={}) {
    Target target; target.at=at; target.airDensity=0;
    target.bodyToEcef={V3{1,0,0},V3{0,1,0},V3{0,0,1}};
    return target;
}
Airframe sceneSolid(double x) {
    Airframe frame;
    Airframe::Component component; component.centre={x,0,0}; component.half={.1,.1,.1};
    frame.components.push_back(component); return frame;
}
FragmentFieldV1 singleSceneProjectile() {
    FragmentFieldV1 field; field.valid=true; field.caseKg=.01;
    FragmentFieldV1::Projectile p;p.velocity={1000,0,0};p.massKg=.01;p.areaM2=1e-5;p.cd=1.3;
    field.projectiles.push_back(p);
    return field;
}
Airframe interiorSkinFrame(bool component) {
    Airframe frame;frame.sections.resize(1);frame.sections[0].skinAreaM2=24;
    frame.sections[0].skinKg=.02*2780*24; // Authored 20 mm aluminium skin.
    Airframe::Panel skin;skin.section=0;skin.surface=false;skin.half={1,1,1};skin.areaM2=24;
    frame.panels.push_back(skin);
    if(component) {Airframe::Component item;item.kind="fuel";item.tank=0;item.half={.2,.2,.2};frame.components.push_back(item);}
    return frame;
}
FragmentFieldV1 interiorRod(double x=0,bool rod=true) {
    auto field=singleSceneProjectile();field.applyBlast=false;
    auto& p=field.projectiles.front();p.at={x,-.05,0};p.velocity={100,0,0};p.segment=rod;
    // Invert the declared THOR normal-impact loss law for the stopping area,
    // then prescribe twice that area. This constructs an opaque SYNTHETIC
    // crossing independently of its observed outcome; no card/material input
    // or production limit is changed. Units: inches, grains, ft/s.
    const double speedFps=100/.3048,massGrains=p.massKg/6.479891e-5,plateIn=.02/.0254;
    const double stopAreaIn2=std::pow(speedFps/(std::pow(10.,7.047)*std::pow(massGrains,-1.072)*
        std::pow(speedFps,-.139)),1/1.029)/plateIn;
    p.areaM2=2*stopAreaIn2*.0254*.0254;
    p.end={x,.05,0};p.endVelocity=p.velocity;return field;
}
Airframe authoredShell(bool partitioned,bool component=false,double componentX=.5,double meshHalfWidth=1) {
    // Two longitudinal ownership volumes describe the SAME physical shell.
    // Their shared axial caps close the contact volumes, not aluminium plates.
    Airframe frame;frame.sections.resize(partitioned?2:1);
    std::vector<ContactMesh> meshes;
    for(std::size_t i=0;i<frame.sections.size();++i) {
        auto& section=frame.sections[i];section.hasRoot=true;
        section.skinAreaM2=partitioned?12:24;section.skinKg=.02*2780*section.skinAreaM2;
        Airframe::Panel panel;panel.section=static_cast<int>(i);panel.surface=false;
        panel.centre={partitioned?(i==0?-.5:.5):0,0,0};panel.half={partitioned?.5:1,1,1};
        panel.areaM2=section.skinAreaM2;frame.panels.push_back(panel);
        const double lo=panel.centre.x-panel.half.x,hi=panel.centre.x+panel.half.x,w=meshHalfWidth;
        ContactMesh mesh;mesh.index=i;
        mesh.vertices={{lo,-w,-w},{hi,-w,-w},{hi,w,-w},{lo,w,-w},
                       {lo,-w,w},{hi,-w,w},{hi,w,w},{lo,w,w}};
        mesh.triangles={{{0,4,7}},{{0,7,3}},{{1,2,6}},{{1,6,5}},
                        {{0,1,5}},{{0,5,4}},{{3,7,6}},{{3,6,2}},
                        {{0,3,2}},{{0,2,1}},{{4,5,6}},{{4,6,7}}};
        mesh.triangleNeighbors.resize(mesh.triangles.size());
        if(partitioned)for(std::size_t t=(i==0?2:0);t<(i==0?4:2);++t)
            mesh.triangleNeighbors[t]=ContactMesh::Neighbor{ContactMesh::Kind::Panel,1-i};
        meshes.push_back(std::move(mesh));
    }
    if(component) {Airframe::Component item;item.kind="fuel";item.tank=0;item.section=componentX<0?0:static_cast<int>(frame.sections.size()-1);
        item.centre={componentX,0,0};item.half={.1,.1,.1};frame.components.push_back(item);}
    std::string error;REQUIRE(setContactGeometryV1(frame,std::move(meshes),error));REQUIRE(error.empty());return frame;
}
}

TEST_CASE("finite rod born inside closed skin reaches its interior component before any skin crossing", "[damage][scene-effects][rod-interior-skin]") {
    const auto frame=interiorSkinFrame(true);const auto target=sceneTarget();
    SceneEnvironmentV1 environment;environment.airDensity=0;environment.maxFlightTimeS=.05;
    // The line is strictly inside both boxes at birth. Neither material point
    // has crossed a skin. Point and finite-line sources must both transfer the
    // independently known 50 J to the component at zero flight time.
    for(const bool rod:{false,true}) {
        INFO("finite line="<<rod);
        const auto result=assessSceneV1(interiorRod(0,rod),{{1,AttachedGeometryView(frame),target}},environment);
        REQUIRE(result.valid);REQUIRE(result.victims.size()==1);const auto& impact=result.victims[0].impact;
        REQUIRE(impact.components.size()==1);CHECK(impact.components[0].hits==1);
        CHECK(std::abs(impact.components[0].energyJ-50)<1e-12);
        CHECK(impact.panels.empty());REQUIRE(result.firstStrikeAgeS);CHECK(*result.firstStrikeAgeS==0);
        CHECK(std::abs(result.absorbedMassKg-.01)<1e-15);
    }
}

TEST_CASE("finite rod initially inside closed skin first loads its actual exit face", "[damage][scene-effects][rod-interior-skin]") {
    const auto frame=interiorSkinFrame(false);SceneEnvironmentV1 environment;environment.airDensity=0;environment.maxFlightTimeS=.05;
    const auto field=interiorRod();REQUIRE(thorResidual(.02,field.projectiles[0].areaM2,.01,100,0)==0);
    const auto result=assessSceneV1(field,{{1,AttachedGeometryView(frame),sceneTarget()}},environment);
    REQUIRE(result.valid);REQUIRE(result.firstStrikeAgeS);
    CHECK(std::abs(*result.firstStrikeAgeS-.01)<1e-12); // (1 m - 0 m) / 100 m/s.
    REQUIRE(result.victims[0].impact.panels.size()==1);CHECK(result.victims[0].impact.panels[0].hits==1);
    CHECK(result.victims[0].impact.components.empty());CHECK(std::abs(result.absorbedMassKg-.01)<1e-15);
}

TEST_CASE("finite rod outside closed skin retains the actual entry crossing", "[damage][scene-effects][rod-interior-skin]") {
    const auto frame=interiorSkinFrame(false);SceneEnvironmentV1 environment;environment.airDensity=0;environment.maxFlightTimeS=.05;
    const auto result=assessSceneV1(interiorRod(-2),{{1,AttachedGeometryView(frame),sceneTarget()}},environment);
    REQUIRE(result.valid);REQUIRE(result.firstStrikeAgeS);
    CHECK(std::abs(*result.firstStrikeAgeS-.01)<1e-12); // (-1 m - -2 m) / 100 m/s.
    REQUIRE(result.victims[0].impact.panels.size()==1);CHECK(result.victims[0].impact.panels[0].hits==1);
    CHECK(std::abs(result.absorbedMassKg-.01)<1e-15);
}

TEST_CASE("finite rod at a closed skin boundary pays for that crossing only once", "[damage][scene-effects][rod-interior-skin]") {
    const auto frame=interiorSkinFrame(false);SceneEnvironmentV1 environment;environment.airDensity=0;environment.maxFlightTimeS=.001;
    for(const double x:{-1.,1.}) {
        INFO("entry boundary -1 / exit boundary +1: "<<x);
        auto field=interiorRod(x);auto& p=field.projectiles[0];p.areaM2/=32;
        const double residual=thorResidual(.02,p.areaM2,p.massKg,100,0);REQUIRE(residual>0);REQUIRE(residual<100);
        const auto result=assessSceneV1(field,{{1,AttachedGeometryView(frame),sceneTarget()}},environment);
        REQUIRE(result.valid);REQUIRE(result.firstStrikeAgeS);CHECK(*result.firstStrikeAgeS==0);
        REQUIRE(result.victims[0].impact.panels.size()==1);CHECK(result.victims[0].impact.panels[0].hits==1);
        REQUIRE(result.continuation.projectiles.size()==1);
        CHECK(std::abs(result.continuation.projectiles[0].velocity.x-residual)<1e-12);
        CHECK(std::abs(result.transferredMomentumEcef.x-p.massKg*(100-residual))<1e-12);
    }
}

TEST_CASE("finite rod already straddling closed skin retains its actual initial overlap", "[damage][scene-effects][rod-interior-skin]") {
    const auto frame=interiorSkinFrame(false);SceneEnvironmentV1 environment;environment.airDensity=0;environment.maxFlightTimeS=.002;
    auto field=interiorRod();auto& p=field.projectiles[0];p.at={-2,0,0};p.end={0,0,0};p.velocity=p.endVelocity={0,100,0};
    const auto result=assessSceneV1(field,{{1,AttachedGeometryView(frame),sceneTarget()}},environment);
    REQUIRE(result.valid);REQUIRE(result.firstStrikeAgeS);CHECK(*result.firstStrikeAgeS==0);
    REQUIRE(result.victims[0].impact.panels.size()==1);CHECK(result.victims[0].impact.panels[0].hits==1);
    CHECK(std::abs(result.absorbedMassKg-.01)<1e-15);
}

TEST_CASE("transmitting finite rod crosses one entry and one exit without a third charge", "[damage][scene-effects][rod-interior-skin]") {
    const auto frame=interiorSkinFrame(false);SceneEnvironmentV1 environment;environment.airDensity=0;environment.maxFlightTimeS=.05;
    auto field=interiorRod(-2);auto& p=field.projectiles[0];p.areaM2/=32;
    const double first=thorResidual(.02,p.areaM2,p.massKg,100,0),second=thorResidual(.02,p.areaM2,p.massKg,first,0);
    REQUIRE(second>0);const auto result=assessSceneV1(field,{{1,AttachedGeometryView(frame),sceneTarget()}},environment);
    REQUIRE(result.valid);REQUIRE(result.firstStrikeAgeS);CHECK(std::abs(*result.firstStrikeAgeS-.01)<1e-12);
    REQUIRE(result.victims[0].impact.panels.size()==1);CHECK(result.victims[0].impact.panels[0].hits==2);
    REQUIRE(result.continuation.projectiles.size()==1);CHECK(std::abs(result.continuation.projectiles[0].velocity.x-second)<1e-12);
    CHECK(std::abs(result.transferredMomentumEcef.x-p.massKg*(100-second))<1e-12);
}

TEST_CASE("authored body ownership partitions do not shield interior components", "[damage][scene-effects][authored-skin]") {
    SceneEnvironmentV1 environment;environment.airDensity=0;environment.maxFlightTimeS=.05;
    for(const bool rod:{false,true})for(const double componentX:{-.5,.5})for(const bool partitioned:{false,true}) {
        INFO("rod="<<rod<<" component x="<<componentX<<" partitioned="<<partitioned);
        const auto frame=authoredShell(partitioned,true,componentX);const auto field=interiorRod(-.5,rod);
        REQUIRE(thorResidual(.02,field.projectiles[0].areaM2,.01,100,0)==0);
        const auto result=assessSceneV1(field,{{1,AttachedGeometryView(frame),sceneTarget()}},environment);
        REQUIRE(result.valid);REQUIRE(result.victims.size()==1);const auto& impact=result.victims[0].impact;
        CHECK(impact.panels.empty());REQUIRE(impact.components.size()==1);CHECK(impact.components[0].component==0);
        CHECK(impact.components[0].hits==1);CHECK(std::abs(impact.components[0].energyJ-50)<1e-12);
        REQUIRE(result.firstStrikeAgeS);CHECK(std::abs(*result.firstStrikeAgeS-(componentX<0?0:.009))<1e-12);
        CHECK(std::abs(result.absorbedMassKg-.01)<1e-15);
    }
}

TEST_CASE("an exposed ownership cut does not become a ballistic skin plate", "[damage][scene-effects][authored-skin]") {
    auto frame=std::make_shared<Airframe>(authoredShell(true));VehicleDamage state(frame);REQUIRE(state.separate(1));
    const auto geometry=state.attachedGeometry();REQUIRE(geometry.panelIndices()==std::vector<std::size_t>{0});
    SceneEnvironmentV1 environment;environment.airDensity=0;environment.maxFlightTimeS=.02;
    for(const bool rod:{false,true}) {
        INFO("rod="<<rod);const auto field=interiorRod(-.5,rod);
        const auto result=assessSceneV1(field,{{1,geometry,sceneTarget()}},environment);
        REQUIRE(result.valid);CHECK(result.victims[0].impact.panels.empty());CHECK_FALSE(result.firstStrikeAgeS);
        CHECK(result.absorbedMassKg==0);CHECK(std::abs(result.remainingMassKg-.01)<1e-15);
        REQUIRE(result.continuation.projectiles.size()==1);CHECK(result.continuation.projectiles[0].velocity.x==100);
    }
}

TEST_CASE("authored shell entry and exit each charge one physical skin crossing", "[damage][scene-effects][authored-skin]") {
    SceneEnvironmentV1 environment;environment.airDensity=0;environment.maxFlightTimeS=.05;
    for(const bool rod:{false,true})for(const bool partitioned:{false,true}) {
        INFO("rod="<<rod<<" partitioned="<<partitioned);const auto frame=authoredShell(partitioned);
        auto field=interiorRod(-2,rod);auto& p=field.projectiles[0];p.areaM2/=32;
        // The point sits on the triangulation diagonal; the finite rod straddles
        // it. A physical face remains one plate, even with two triangle hits.
        p.at.y=rod?-.05:0;
        const double first=thorResidual(.02,p.areaM2,p.massKg,100,0),second=thorResidual(.02,p.areaM2,p.massKg,first,0);
        REQUIRE(second>0);const auto result=assessSceneV1(field,{{1,AttachedGeometryView(frame),sceneTarget()}},environment);
        REQUIRE(result.valid);REQUIRE(result.firstStrikeAgeS);CHECK(std::abs(*result.firstStrikeAgeS-.01)<1e-12);
        int hits=0;for(const auto& hit:result.victims[0].impact.panels) {hits+=hit.hits;CHECK(hit.panel>=0);CHECK(hit.panel<static_cast<int>(frame.panels.size()));}
        CHECK(hits==2);REQUIRE(result.continuation.projectiles.size()==1);
        CHECK(std::abs(result.continuation.projectiles[0].velocity.x-second)<1e-12);
        CHECK(std::abs(result.transferredMomentumEcef.x-p.massKg*(100-second))<1e-12);
    }
}

TEST_CASE("authored skin triangles locate a crossing inside its broader damage box", "[damage][scene-effects][authored-skin]") {
    const auto frame=authoredShell(false,false,.5,.5);SceneEnvironmentV1 environment;environment.airDensity=0;environment.maxFlightTimeS=.05;
    for(const bool rod:{false,true}) {
        INFO("rod="<<rod);auto field=interiorRod(0,rod);auto& p=field.projectiles[0];
        p.at={-.05,-2,0};p.end={.05,-2,0};p.velocity=p.endVelocity={0,100,0};
        const auto result=assessSceneV1(field,{{1,AttachedGeometryView(frame),sceneTarget()}},environment);
        REQUIRE(result.valid);REQUIRE(result.firstStrikeAgeS);CHECK(std::abs(*result.firstStrikeAgeS-.015)<1e-12);
        REQUIRE(result.victims[0].impact.panels.size()==1);CHECK(result.victims[0].impact.panels[0].panel==0);
        CHECK(result.victims[0].impact.panels[0].hits==1);CHECK(std::abs(result.absorbedMassKg-.01)<1e-15);
    }
}

TEST_CASE("one authored panel retains every crossing through multiple shell regions", "[damage][scene-effects][authored-skin]") {
    auto frame=authoredShell(false);ContactMesh mesh=*frame.contactGeometry->panel(0);
    const auto original=mesh;
    for(auto& vertex:mesh.vertices)vertex.x-=2;
    for(auto vertex:original.vertices){vertex.x+=2;mesh.vertices.push_back(vertex);}
    for(auto triangle:original.triangles){for(auto& vertex:triangle)vertex+=original.vertices.size();mesh.triangles.push_back(triangle);}
    mesh.triangleNeighbors.resize(mesh.triangles.size());frame.panels[0].half.x=3;
    std::string error;REQUIRE(setContactGeometryV1(frame,{mesh},error));REQUIRE(error.empty());
    SceneEnvironmentV1 environment;environment.airDensity=0;environment.maxFlightTimeS=.1;
    for(const bool rod:{false,true}) {
        INFO("rod="<<rod);auto field=interiorRod(-4,rod);auto& p=field.projectiles[0];p.areaM2/=32768;
        double residual=100;for(int i=0;i<4;++i)residual=thorResidual(.02,p.areaM2,p.massKg,residual,0);
        REQUIRE(residual>0);const auto result=assessSceneV1(field,{{1,AttachedGeometryView(frame),sceneTarget()}},environment);
        REQUIRE(result.valid);REQUIRE(result.victims[0].impact.panels.size()==1);
        CHECK(result.victims[0].impact.panels[0].hits==4);REQUIRE(result.continuation.projectiles.size()==1);
        CHECK(std::abs(result.continuation.projectiles[0].velocity.x-residual)<1e-12);
    }
}

TEST_CASE("one scene projectile cannot be independently absorbed by two victims", "[damage][scene-effects]") {
    const auto front=sceneSolid(1),back=sceneSolid(2); const auto target=sceneTarget();
    const std::vector<SceneVictimV1> victims={{22,AttachedGeometryView(back),target},{11,AttachedGeometryView(front),target}};
    SceneEnvironmentV1 environment; environment.airDensity=0;
    const auto field=singleSceneProjectile();
    const auto result=assessSceneV1(field,victims,environment);
    REQUIRE(result.valid); REQUIRE(result.victims.size()==2);
    CHECK(result.victims[0].identity==11); REQUIRE(result.victims[0].impact.components.size()==1);
    CHECK(result.victims[1].identity==22); CHECK(result.victims[1].impact.components.empty());
    CHECK(std::abs(result.victims[0].impact.components[0].energyJ-5000)<1e-9);
    CHECK(std::abs(result.transferredMomentumEcef.x-10)<1e-12);
    auto reverse=victims;std::reverse(reverse.begin(),reverse.end());
    const auto repeated=assessSceneV1(field,reverse,environment);
    REQUIRE(repeated.valid); CHECK(repeated.victims[0].impact.components[0].energyJ==result.victims[0].impact.components[0].energyJ);
    CHECK(repeated.victims[1].impact.components.empty());
}

TEST_CASE("scene residual skin penetration transfers a single finite momentum budget", "[damage][scene-effects]") {
    Airframe shield;shield.sections.resize(1);shield.sections[0].skinKg=.00278;shield.sections[0].skinAreaM2=1;
    Airframe::Panel panel;panel.section=0;panel.surface=false;panel.centre={1,0,0};panel.half={.01,1,1};shield.panels.push_back(panel);
    const auto solid=sceneSolid(2);const auto target=sceneTarget();SceneEnvironmentV1 environment;environment.airDensity=0;
    const auto result=assessSceneV1(singleSceneProjectile(),{{1,AttachedGeometryView(shield),target},{2,AttachedGeometryView(solid),target}},environment);
    REQUIRE(result.valid);REQUIRE(result.victims[1].impact.components.size()==1);
    const double energy=result.victims[1].impact.components[0].energyJ;
    CHECK(energy>0);CHECK(energy<5000);
    CHECK(std::abs(result.transferredMomentumEcef.x-10)<1e-10);
    CHECK(std::abs(result.remainingMassKg)<1e-12);
    CHECK(result.airMomentumEcef.x==0);
}

TEST_CASE("scene fragment interception is Galilean invariant for translating targets", "[damage][scene-effects]") {
    const auto solid=sceneSolid(2);auto target=sceneTarget();SceneEnvironmentV1 environment;environment.airDensity=0;
    auto field=singleSceneProjectile();
    const auto first=assessSceneV1(field,{{1,AttachedGeometryView(solid),target}},environment);
    const V3 boost{250,-40,30};field.projectiles[0].velocity+=boost;target.velocity+=boost;environment.airVelocity+=boost;
    const auto second=assessSceneV1(field,{{1,AttachedGeometryView(solid),target}},environment);
    REQUIRE(first.valid);REQUIRE(second.valid);REQUIRE(second.victims[0].impact.components.size()==1);
    CHECK(std::abs(first.victims[0].impact.components[0].energyJ-second.victims[0].impact.components[0].energyJ)<1e-9);
    CHECK(length(first.transferredMomentumEcef-second.transferredMomentumEcef)<1e-12);
}

TEST_CASE("scene angular impulse is invariant under a common translational boost", "[damage][scene-effects][scene-moving-origin-moment]") {
    const auto solid=sceneSolid(2);SceneEnvironmentV1 environment;environment.airDensity=0;
    const auto stationary=assessSceneV1(singleSceneProjectile(),{{1,AttachedGeometryView(solid),sceneTarget()}},environment);
    REQUIRE(stationary.valid);REQUIRE(stationary.victims[0].impact.components.size()==1);
    CHECK(length(stationary.victims[0].impact.angularImpulseBody)<1e-12);
    // The ray passes through the moving body's origin. Changing only the
    // inertial reference velocity cannot manufacture a spin impulse.
    for(const V3 boost: {V3{250,-40,30},V3{-100,75,-60}}) {
        auto field=singleSceneProjectile();field.projectiles[0].velocity+=boost;
        auto target=sceneTarget();target.velocity=boost;
        const auto result=assessSceneV1(field,{{1,AttachedGeometryView(solid),target}},environment);
        REQUIRE(result.valid);REQUIRE(result.victims[0].impact.components.size()==1);
        CHECK(length(result.victims[0].impact.impulseBody-stationary.victims[0].impact.impulseBody)<1e-12);
        CHECK(length(result.victims[0].impact.angularImpulseBody-stationary.victims[0].impact.angularImpulseBody)<1e-12);
    }
}

TEST_CASE("captured translating scene moments use the material lever at contact independently of batch duration", "[damage][scene-effects][scene-moving-origin-moment]") {
    auto solid=sceneSolid(2);solid.components[0].centre={2,.5,-.25};
    const V3 origin{5,-3,2},offset{0,.5,-.25},impulse{10,0,0};
    // The first box face is x=1.9 in structural coordinates. Its unknown
    // contact-time x does not enter this cross product because J is axial.
    const V3 expected=cross(offset,impulse);
    for(const double duration: {.002,.01,.04})for(const V3 boost: {V3{},V3{250,-40,30},V3{-100,75,-60}}) {
        CAPTURE(duration,boost.x,boost.y,boost.z);
        auto field=singleSceneProjectile();field.projectiles[0].at=origin+offset;field.projectiles[0].velocity+=boost;
        auto target=sceneTarget(origin);target.velocity=boost;
        SceneVictimV1 victim{1,AttachedGeometryView(solid),target};
        for(const double time: {0.,duration/2,duration}) {
            auto sample=target;sample.at=origin+boost*time;victim.motion.push_back({time,sample});
        }
        SceneEnvironmentV1 environment;environment.airDensity=0;environment.maxFlightTimeS=duration;
        const auto result=assessSceneV1(field,{victim},environment);
        REQUIRE(result.valid);REQUIRE(result.victims[0].impact.components.size()==1);
        REQUIRE(result.firstStrikeAgeS);CHECK(std::abs(*result.firstStrikeAgeS-.0019)<1e-12);
        CHECK(length(result.victims[0].impact.impulseBody-impulse)<1e-11);
        CHECK(length(result.victims[0].impact.angularImpulseBody-expected)<1e-11);
    }
}

TEST_CASE("finite rod material between endpoints can strike a narrow component", "[damage][scene-effects][connected-rod]") {
    auto field=singleSceneProjectile();auto& rod=field.projectiles[0];rod.segment=true;
    rod.at={0,-1,0};rod.end={0,1,0};rod.velocity={1000,0,0};rod.endVelocity=rod.velocity;
    const auto solid=sceneSolid(2);SceneEnvironmentV1 environment;environment.airDensity=0;
    const auto result=assessSceneV1(field,{{1,AttachedGeometryView(solid),sceneTarget()}},environment);
    REQUIRE(result.valid);REQUIRE(result.victims[0].impact.components.size()==1);
    CHECK(result.victims[0].impact.components[0].hits==1);
    CHECK(std::abs(result.transferredMomentumEcef.x-10)<1e-12);
}

TEST_CASE("detached damage state owns only its original section identities", "[damage][scene-effects][detached-effects]") {
    auto frame=std::make_shared<Airframe>();frame->sections.resize(2);
    for(auto& section:frame->sections)section.hasRoot=true;
    for(int section:{-1,0,1}) {Airframe::Component component;component.section=section;frame->components.push_back(component);}
    VehicleDamage parent(frame);REQUIRE(parent.separate(1));VehicleDamage detached(frame,1);
    const auto parentView=parent.attachedGeometry(),detachedView=detached.attachedGeometry();
    REQUIRE(parentView.componentIndices()==std::vector<std::size_t>{0,1});
    REQUIRE(detachedView.componentIndices()==std::vector<std::size_t>{2});
    CHECK(parent.airframe().components.size()==3);CHECK(detached.airframe().components.size()==3);
}

TEST_CASE("scene atmospheric momentum and retained projectile momentum close independently", "[damage][scene-effects]") {
    const auto field=singleSceneProjectile();SceneEnvironmentV1 environment;
    environment.airDensity=1.2;environment.maxFlightTimeS=.05;
    const auto result=assessSceneV1(field,{},environment);REQUIRE(result.valid);
    const double coefficient=1.2*1.3*1e-5/(2*.01);
    const double expectedSpeed=1000/(1+coefficient*1000*.05);
    CHECK(std::abs(result.remainingMomentumEcef.x-.01*expectedSpeed)<1e-10);
    CHECK(std::abs(result.remainingMomentumEcef.x+result.airMomentumEcef.x-10)<1e-10);
    CHECK(result.transferredMomentumEcef.x==0);CHECK(result.remainingMassKg==.01);CHECK(result.absorbedMassKg==0);
}

TEST_CASE("scene selection rejects duplicate identity and malformed populations atomically", "[damage][scene-effects]") {
    const auto solid=sceneSolid(1);const auto target=sceneTarget();auto field=singleSceneProjectile();SceneEnvironmentV1 environment;
    CHECK_FALSE(assessSceneV1(field,{{1,AttachedGeometryView(solid),target},{1,AttachedGeometryView(solid),target}},environment).valid);
    field.projectiles[0].massKg*=2;
    const auto result=assessSceneV1(field,{{1,AttachedGeometryView(solid),target}},environment);
    CHECK_FALSE(result.valid);CHECK(result.victims.empty());
}

TEST_CASE("a resumed field cannot apply a future hit or resample surviving material", "[damage][scene-effects][causal-effects]") {
    const auto solid=sceneSolid(1);const auto target=sceneTarget();SceneEnvironmentV1 environment;
    environment.airDensity=0;environment.maxFlightTimeS=.0005;
    const auto first=assessSceneV1(singleSceneProjectile(),{{1,AttachedGeometryView(solid),target}},environment);
    REQUIRE(first.valid);CHECK(first.victims[0].impact.components.empty());CHECK(first.transferredMomentumEcef.x==0);
    REQUIRE(first.continuation.valid);REQUIRE(first.continuation.projectiles.size()==1);
    CHECK(std::abs(first.continuation.projectiles[0].at.x-.5)<1e-12);
    const auto second=assessSceneV1(first.continuation,{{1,AttachedGeometryView(solid),target}},environment);
    REQUIRE(second.valid);REQUIRE(second.victims[0].impact.components.size()==1);
    CHECK(std::abs(second.victims[0].impact.components[0].energyJ-5000)<1e-9);
    CHECK(second.continuation.projectiles.empty());CHECK(second.absorbedMassKg==.01);
}

TEST_CASE("scene execution refuses nonphysical finite-overflow and unavailable atmosphere", "[damage][scene-effects][effects-quality-boundary]") {
    const auto solid=sceneSolid(1);SceneEnvironmentV1 environment;environment.airDensity=0;
    auto field=singleSceneProjectile();auto target=sceneTarget();
    field.burst.at.x=std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(assessSceneV1(field,{{1,AttachedGeometryView(solid),target}},environment).valid);
    field=singleSceneProjectile();field.projectiles[0].velocity.x=std::numeric_limits<double>::max()/2;
    CHECK_FALSE(assessSceneV1(field,{{1,AttachedGeometryView(solid),target}},environment).valid);
    field=singleSceneProjectile();target.airPressurePa=std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(assessSceneV1(field,{{1,AttachedGeometryView(solid),target}},environment).valid);
    target=sceneTarget();target.bodyToEcef[2].z=-1;
    CHECK_FALSE(assessSceneV1(field,{{1,AttachedGeometryView(solid),target}},environment).valid);
}

TEST_CASE("fragment gravity advances real material and closes its external impulse ledger", "[damage][scene-effects][causal-effects]") {
    auto field=singleSceneProjectile();SceneEnvironmentV1 environment;environment.airDensity=0;
    environment.gravityEcef={0,0,-10};environment.maxFlightTimeS=.1;
    const auto result=assessSceneV1(field,{},environment);REQUIRE(result.valid);REQUIRE(result.continuation.projectiles.size()==1);
    const auto& particle=result.continuation.projectiles.front();
    CHECK(std::abs(particle.at.x-100)<1e-10);CHECK(std::abs(particle.at.z+.05)<1e-10);
    CHECK(std::abs(particle.velocity.z+1)<1e-10);CHECK(std::abs(result.gravityImpulseEcef.z+.01)<1e-12);
    CHECK(length(result.airMomentumEcef)<1e-12);
    CHECK(length(V3{10,0,0}+result.gravityImpulseEcef-result.remainingMomentumEcef)<1e-10);
}

TEST_CASE("fragment interception consumes captured moving origins without extrapolating unavailable history", "[damage][scene-effects][causal-effects]") {
    const auto solid=sceneSolid(1);auto target=sceneTarget();SceneEnvironmentV1 environment;environment.airDensity=0;environment.maxFlightTimeS=.001;
    SceneVictimV1 victim{1,AttachedGeometryView(solid),target};victim.motion.push_back({0,target});
    target.at.y=2;victim.motion.push_back({.0005,target});victim.motion.push_back({.001,target});
    const auto moving=assessSceneV1(singleSceneProjectile(),{victim},environment);REQUIRE(moving.valid);
    CHECK(moving.victims[0].impact.components.empty());
    victim.motion.pop_back();CHECK_FALSE(assessSceneV1(singleSceneProjectile(),{victim},environment).valid);
}

TEST_CASE("resuming at a crossed skin preserves its identity and residual impulse", "[damage][scene-effects][causal-effects]") {
    Airframe shield;shield.sections.resize(1);shield.sections[0].skinKg=.00278;shield.sections[0].skinAreaM2=1;
    Airframe::Panel panel;panel.section=0;panel.surface=false;panel.centre={1,0,0};panel.half={.1,1,1};shield.panels.push_back(panel);
    const auto solid=sceneSolid(1.5);const auto target=sceneTarget();
    const std::vector<SceneVictimV1> victims={{1,AttachedGeometryView(shield),target},{2,AttachedGeometryView(solid),target}};
    SceneEnvironmentV1 environment;environment.airDensity=0;environment.maxFlightTimeS=.002;
    const auto whole=assessSceneV1(singleSceneProjectile(),victims,environment);REQUIRE(whole.valid);
    environment.maxFlightTimeS=.0009;const auto first=assessSceneV1(singleSceneProjectile(),victims,environment);REQUIRE(first.valid);
    environment.maxFlightTimeS=.0011;const auto second=assessSceneV1(first.continuation,victims,environment);REQUIRE(second.valid);
    REQUIRE(second.victims[1].impact.components.size()==1);REQUIRE(whole.victims[1].impact.components.size()==1);
    CHECK(std::abs(second.victims[1].impact.components[0].energyJ-whole.victims[1].impact.components[0].energyJ)<1e-8);
    CHECK(length(first.transferredMomentumEcef+second.transferredMomentumEcef-whole.transferredMomentumEcef)<1e-10);
}

TEST_CASE("absorbing a finite spinning rod transfers its intrinsic angular momentum", "[damage][scene-effects][connected-rod]") {
    auto field=singleSceneProjectile();auto& rod=field.projectiles.front();
    rod.segment=true;rod.at={0,-1,0};rod.end={0,1,0};rod.endVelocity={1000,0,2};
    const auto solid=sceneSolid(1);SceneEnvironmentV1 environment;environment.airDensity=0;environment.maxFlightTimeS=.002;
    const auto result=assessSceneV1(field,{{1,AttachedGeometryView(solid),sceneTarget()}},environment);
    REQUIRE(result.valid);REQUIRE(result.victims[0].impact.components.size()==1);
    const auto torque=result.victims[0].impact.angularImpulseBody;
    CHECK(std::abs(torque.x-.01/3)<1e-10);CHECK(std::abs(torque.y)<1e-10);CHECK(std::abs(torque.z)<1e-10);
    CHECK(result.victims[0].impact.components[0].energyJ>5000);
}

TEST_CASE("every future captured target sample retains complete physical validity", "[damage][scene-effects][effects-quality-boundary]") {
    const auto solid=sceneSolid(1);const auto initial=sceneTarget();SceneEnvironmentV1 environment;
    environment.airDensity=0;environment.maxFlightTimeS=.002;
    for(unsigned invalid=0;invalid<6;++invalid) {
        auto future=initial;
        switch(invalid) {
        case 0: future.bodyToEcef={V3{},V3{},V3{}};break;
        case 1: future.bodyToEcef[2].z=-1;break;
        case 2: future.angularVelocityEcef.x=std::numeric_limits<double>::quiet_NaN();break;
        case 3: future.airPressurePa=std::numeric_limits<double>::quiet_NaN();break;
        case 4: future.airTemperatureK=0;break;
        case 5: future.airDensity=-1;break;
        }
        SceneVictimV1 victim{1,AttachedGeometryView(solid),initial};victim.motion={{0,initial},{.002,future}};
        const auto result=assessSceneV1(singleSceneProjectile(),{victim},environment);
        CHECK_FALSE(result.valid);CHECK(result.victims.empty());CHECK(result.continuation.projectiles.empty());
    }
}

TEST_CASE("finite-line gravity drag and rotating-target interception converges against independent RK4", "[damage][scene-effects][causal-effects][effects-convergence]") {
    constexpr double referenceTime=.01975,duration=.03,density=.5,cd=1.3,area=.01,mass=.01,omega=8;
    const V3 air{0,3,0},gravity{0,0,-9.80665};const double drag=density*cd*area/(2*mass);
    // Independent fixed-step coupled ODE oracle; no production trajectory or
    // intersection helper supplies its expected contact geometry/time.
    V3 position,velocity{100,0,0};constexpr unsigned count=19750;const double dt=referenceTime/count;
    const auto acceleration=[&](V3 v){const auto relative=v-air;return gravity-relative*(drag*length(relative));};
    for(unsigned i=0;i<count;++i) {
        const auto a1=acceleration(velocity),v2=velocity+a1*(dt/2),a2=acceleration(v2);
        const auto v3=velocity+a2*(dt/2),a3=acceleration(v3),v4=velocity+a3*dt,a4=acceleration(v4);
        position+=(velocity+v2*2+v3*2+v4)*(dt/6);velocity+=(a1+a2*2+a3*2+a4)*(dt/6);
    }
    const double support=.1*std::cos(omega*referenceTime)+.15*std::sin(omega*referenceTime);
    const double targetX=position.x+support;
    Airframe frame;Airframe::Component box;box.half={.1,.15,.1};frame.components.push_back(box);
    std::array<double,3> errors{};std::size_t index=0;
    for(const double cadence:{.002,.001,.0005}) {
        auto field=singleSceneProjectile();auto& rod=field.projectiles.front();rod.segment=true;
        rod.at={0,-1,0};rod.end={0,1,0};rod.velocity=rod.endVelocity={100,0,0};rod.areaM2=area;
        auto initial=sceneTarget({targetX,.1,0});initial.velocity={0,.3,0};initial.angularVelocityEcef={0,0,omega};
        SceneVictimV1 victim{1,AttachedGeometryView(frame),initial};
        for(double time=0;;time=std::min(duration,time+cadence)) {
            auto target=initial;target.at.y+=.3*time;const double c=std::cos(omega*time),s=std::sin(omega*time);
            target.bodyToEcef={V3{c,s,0},V3{-s,c,0},V3{0,0,1}};victim.motion.push_back({time,target});if(time==duration)break;
        }
        SceneEnvironmentV1 environment;environment.airDensity=density;environment.airVelocity=air;environment.gravityEcef=gravity;
        environment.maxFlightTimeS=duration;environment.maxSubstepS=cadence;
        const auto result=assessSceneV1(field,{victim},environment);REQUIRE(result.valid);REQUIRE(result.firstStrikeAgeS);
        REQUIRE(result.victims[0].impact.components.size()==1);CHECK(result.absorbedMassKg==mass);
        errors[index++]=std::abs(*result.firstStrikeAgeS-referenceTime);
    }
    CHECK(errors[1]<=errors[0]+1e-9);CHECK(errors[2]<=errors[1]+1e-9);
    CHECK(errors[2]<errors[0]*.75);CHECK(errors[2]<2e-5);
    std::printf("SIM_M03_EFFECTS_CONVERGENCE dt2ms_error_s=%.12g dt1ms_error_s=%.12g dt0.5ms_error_s=%.12g\n",errors[0],errors[1],errors[2]);
}

TEST_CASE("departing body inherits damaged fuel state and translates its original material origin", "[damage][scene-effects][detached-effects]") {
    auto frame=std::make_shared<Airframe>();frame->sections.resize(2);for(auto& section:frame->sections)section.hasRoot=true;
    Airframe::Component fuel;fuel.kind="fuel";fuel.section=1;fuel.tank=7;fuel.fuelKg=42;fuel.centre={6,2,1};frame->components.push_back(fuel);
    VehicleDamage parent(frame);Impact impact;Impact::ComponentHits hit;hit.component=0;hit.hits=1;hit.energyJ=1e9;hit.areaM2=.01;hit.fastestMs=1500;
    impact.components.push_back(hit);fsim::Rng rng(17);parent.apply(impact,101325,rng);REQUIRE(parent.tanks().at(7).burning);
    REQUIRE(parent.separate(1));auto body=parent.detachedState(1,{5,0,0});REQUIRE(body);
    CHECK(body->tanks().at(7).burning);CHECK(body->tanks().at(7).holeM2==parent.tanks().at(7).holeM2);
    CHECK(body->airframe().components[0].centre.x==1);CHECK(parent.airframe().components[0].centre.x==6);
    REQUIRE(body->setDetachedFuelKg(7,12));CHECK(body->removeDetachedFuelKg(7,3)==3);
    REQUIRE(body->detachedFuelKg(7));CHECK(*body->detachedFuelKg(7)==9);CHECK_FALSE(parent.detachedFuelKg(7));
    CHECK(body->removeDetachedFuelKg(7,100)==9);CHECK(*body->detachedFuelKg(7)==0);
    CHECK_FALSE(parent.detachedState(0));
}

TEST_CASE("A fixed burst point keeps its location and velocity when the CG reference moves", "[damage][material-point]") {
    fsim::sim::VehicleState s{};
    for (int i=0;i<9;++i) s.rotationBodyToEcef[i]=(i%4==0 ? 1.0 : 0.0);
    s.positionEcef[0]=100; s.positionEcef[1]=200; s.positionEcef[2]=300;
    s.velocityBodyMs[0]=10; s.velocityBodyMs[1]=20; s.velocityBodyMs[2]=30;
    s.angularRateBodyRadS[2]=2;
    const auto first=fsim::ms::weapon::materialPoint(s,{1,0,0},{3,0,0});
    REQUIRE(first.valid);
    CHECK(first.position.x==102); CHECK(first.position.y==200); CHECK(first.position.z==300);
    CHECK(first.velocity.x==10); CHECK(first.velocity.y==24); CHECK(first.velocity.z==30);
    s.positionEcef[0]+=1; s.velocityBodyMs[1]+=2;
    const auto moved=fsim::ms::weapon::materialPoint(s,{2,0,0},{3,0,0});
    REQUIRE(moved.valid);
    CHECK(length(moved.position-first.position)==0); CHECK(length(moved.velocity-first.velocity)==0);
    CHECK(moved.angularVelocity.z==2);
}

TEST_CASE("A mechanical impulse loads the root without inventing fragment hits", "[damage][contact-energy]") {
    auto frame = std::make_shared<Airframe>();
    frame->nUltimate[1] = 9;
    frame->sections.resize(1);
    auto& section = frame->sections[0];
    section.hasRoot = true;
    section.axes = {{0, 1, 0}};
    section.strengthNm = {1};
    Airframe::Panel face;
    face.section = 0;
    face.corners = {{{1, -1, -1}, {1, 1, -1}, {1, 1, 1}, {1, -1, 1}}};
    frame->panels.push_back(face);
    VehicleDamage damage(frame);
    Impact impact;
    Impact::PanelHits load;
    load.panel = 0;
    load.momentumNs = {0, 0, 100};
    impact.panels.push_back(load);
    fsim::Rng rng(19);
    const auto before = rng.state();
    const auto changes = damage.apply(impact, 101325, rng);
    REQUIRE(damage.sections()[0].kick > 1);
    REQUIRE(damage.sections()[0].separated);
    CHECK(std::any_of(changes.begin(), changes.end(), [](const Change& c) { return c.kind == Change::Kind::Separated; }));
    CHECK(impact.fragmentsThrown == 0);
    CHECK(impact.fragmentsStriking == 0);
    CHECK(damage.sections()[0].holeM2 == 0);
    CHECK(rng.state() == before);
}

TEST_CASE("A localized contact uses its real point rather than a damage-panel midpoint", "[damage][contact-energy]") {
    auto frame=std::make_shared<Airframe>(); frame->nUltimate[1]=9; frame->sections.resize(1);
    auto& section=frame->sections[0]; section.hasRoot=true; section.axes={{0,1,0}}; section.strengthNm={1};
    Airframe::Panel face; face.section=0;
    face.corners={{{10,-1,-1},{10,1,-1},{10,1,1},{10,-1,1}}}; frame->panels.push_back(face);
    VehicleDamage damage(frame); fsim::Rng rng(5); const auto before=rng.state();
    const auto changes=damage.apply(Impact{},101325,rng,{{0,section.root,{0,0,1e6}}});
    CHECK(changes.empty()); CHECK(damage.sections()[0].kick==0); CHECK_FALSE(damage.sections()[0].separated);
    CHECK(damage.sections()[0].holeM2==0); CHECK(rng.state()==before);
}

TEST_CASE("Departed components cannot keep operating on the parent aircraft", "[damage][contact-energy]") {
    auto frame=std::make_shared<Airframe>(); frame->sections.resize(2);
    for (auto& section:frame->sections) section.hasRoot=true;
    Airframe::Component crew; crew.section=0; crew.kind="crew"; frame->components.push_back(crew);
    Airframe::Component engine; engine.section=0; engine.kind="engine"; engine.engine=0; frame->components.push_back(engine);
    Airframe::Component control; control.section=0; control.kind="control"; control.channel="elevator"; frame->components.push_back(control);
    engine.section=1; engine.engine=1; frame->components.push_back(engine);
    VehicleDamage damage(frame);
    CHECK(damage.detachComponents(0).empty()); CHECK_FALSE(damage.crewLost());
    REQUIRE(damage.separate(0));
    const auto changes=damage.detachComponents(0);
    CHECK(changes.size()==3); CHECK(damage.crewLost()); CHECK(damage.engineOut(0)); CHECK_FALSE(damage.engineOut(1));
    REQUIRE(damage.lostChannels().size()==1); CHECK(damage.lostChannels()[0]=="elevator");
    CHECK(damage.detachComponents(0).empty());
}

namespace {

const std::filesystem::path kAircraft = FSIM_TEST_AIRCRAFT_DIR;
const std::filesystem::path kStores = kAircraft.parent_path() / "assets" / "stores";

bool near(double a, double b, double tol) { return std::abs(a - b) <= tol; }

std::shared_ptr<const StoreCard> card(const char* id) {
    std::vector<std::string> errors;
    auto c = readStoreCard(kStores / (std::string(id) + ".json"), errors);
    INFO((errors.empty() ? std::string() : errors.front()));
    REQUIRE(errors.empty());
    return c;
}

// Kinney and Graham's free-air fits (Explosive Shocks in Air, 1985): independent of Kingery-Bulmash's
double kinneyPressure(double z) {
    return 101325.0 * 808.0 * (1.0 + (z / 4.5) * (z / 4.5)) /
           (std::sqrt(1.0 + (z / 0.048) * (z / 0.048)) * std::sqrt(1.0 + (z / 0.32) * (z / 0.32)) * std::sqrt(1.0 + (z / 1.35) * (z / 1.35)));
}
double kinneyImpulse(double z, double w) { // Pa s
    return 1e5 * 1e-3 * 0.067 * std::sqrt(1.0 + std::pow(z / 0.23, 4.0)) / (z * z * std::cbrt(1.0 + std::pow(z / 1.55, 3.0))) * std::cbrt(w);
}

} // namespace

TEST_CASE("a warhead's fragments: Gurney's velocity, the rule's case and fragments, from each card", "[damage]") {
    // Gurney's cylinder: M/C = 1 throws at sqrt(2E) / sqrt(1.5)
    CHECK(near(gurneyCylinder(2700.0, 1.0, 1.0), 2700.0 / std::sqrt(1.5), 1e-9));
    CHECK(gurneyCylinder(2700.0, 1.0, 0.0) == 0.0);
    // The explicitly assumed generic profile on this card retains the original
    // mass inputs. These checks do not establish measured WDU-41/B performance.
    const auto aim120 = warheadOf(*card("aim-120c"));
    REQUIRE(aim120);
    CHECK(aim120->kind == Warhead::Kind::Fragmentation);
    CHECK(near(aim120->caseKg, 0.85 * (20.0 - 7.17), 1e-9));
    CHECK(aim120->speedMs > 1600.0);
    CHECK(aim120->speedMs < 2100.0);
    CHECK(aim120->lengthM > 0.15);
    CHECK(aim120->lengthM < 0.6);
    CHECK(aim120->fragmentKg > 0.002);
    CHECK(aim120->fragmentKg < 0.02);
    CHECK(aim120->fragments > 500);
    CHECK(near(aim120->fragments * aim120->fragmentKg, aim120->caseKg, 0.01 * aim120->caseKg));
    CHECK(aim120->tntKg > aim120->explosiveKg); // the card's declared uncalibrated blast proxy
    REQUIRE(aim120->modelVersion == 2);
    // The selected V2 retains finite mass/origin construction while exposing
    // all seven sectors of its pinned generic game-reference angular field.
    REQUIRE(aim120->zones.size()==7);
    CHECK(aim120->zones.front().fromDeg==0);CHECK(aim120->zones.front().toDeg==20);
    CHECK(aim120->zones.front().share==.0375);
    CHECK(aim120->zones.back().fromDeg==160);CHECK(aim120->zones.back().toDeg==180);
    CHECK(aim120->zones.back().share==.0125);
    CHECK(near(aim120->fragments * aim120->fragmentKg, aim120->caseKg, 1e-12));
    // the R-27's rods: fewer, heavier, slower (a continuous rod's speed at most Talos's)
    const auto r27 = warheadOf(*card("r-27er"));
    REQUIRE(r27);
    CHECK(r27->kind == Warhead::Kind::Rod);
    CHECK(r27->fragmentKg > 10.0 * aim120->fragmentKg);
    CHECK(r27->speedMs <= 1400.0);
    // every missile's card gives one; a bomb's card none yet
    for (const auto& f : std::filesystem::directory_iterator(kStores)) {
        std::vector<std::string> errors;
        const auto c = readStoreCard(f.path(), errors);
        INFO(f.path().string() << (errors.empty() ? std::string() : ": " + errors.front()));
        REQUIRE(c);
        CHECK(errors.empty());
        if (c->storeClass == "aam") {
            const auto w = warheadOf(*c);
            REQUIRE(w);
            CHECK(w->fragments > 10);
            CHECK(w->speedMs > 800.0);
        }
    }
}

TEST_CASE("a burst's blast: Kingery-Bulmash in free air against Kinney-Graham, scaled for altitude", "[damage]") {
    // 2 kg of TNT in free air (the surface fits' 1 kg): the two fits agree within some 12 % from Z = 4 out, within
    // 25 % close in (Z 1.6: the fits part near the fireball, where neither is measured well)
    for (double r : {2.0, 5.0, 10.0}) {
        const Blast b = blastAt(2.0, r, 101325.0, 288.15);
        const double z = r / std::cbrt(2.0), tol = z < 3.0 ? 0.25 : 0.12;
        INFO("r " << r << " m: " << b.overpressurePa << " Pa against " << kinneyPressure(z) << ", " << b.impulsePas << " Pa s against "
                  << kinneyImpulse(z, 2.0));
        CHECK(near(b.overpressurePa, kinneyPressure(z), tol * kinneyPressure(z)));
        CHECK(near(b.impulsePas, kinneyImpulse(z, 2.0), (tol + 0.03) * kinneyImpulse(z, 2.0)));
    }
    // Sachs: in air at half the pressure, the overpressure at the scaled distance a half
    const Blast sea = blastAt(8.0, 6.0, 101325.0, 288.15), high = blastAt(8.0, 6.0 / std::cbrt(0.5), 0.5 * 101325.0, 288.15);
    CHECK(near(high.overpressurePa, 0.5 * sea.overpressurePa, 1e-6 * sea.overpressurePa));
    // the reflected peak: twice a weak shock's, up to eight times a strong one's
    CHECK(near(reflected(10.0, 101325.0), 20.0, 0.01));
    CHECK(reflected(1e9, 101325.0) / 1e9 > 7.9);
}

TEST_CASE("a fragment through the skin: THOR's residual velocity", "[damage]") {
    // a 6 g steel cube (9 mm) at 1.8 km/s through 3 mm of aluminium: on, slower; it slows more through a thicker plate,
    // and obliquely; a slow one through a thick plate stops
    const double m = 0.006, a = 1.5 * 0.009 * 0.009;
    const double thin = thorResidual(0.003, a, m, 1800.0, 0.0);
    CHECK(thin > 1000.0);
    CHECK(thin < 1800.0);
    CHECK(thorResidual(0.008, a, m, 1800.0, 0.0) < thin);
    CHECK(thorResidual(0.003, a, m, 1800.0, 1.0) < thin);
    CHECK(thorResidual(0.02, a, m, 300.0, 0.0) == 0.0);
    CHECK(thorResidual(0.0, a, m, 300.0, 0.0) == 300.0);
}

TEST_CASE("fragment momentum lost to air is not applied to the target", "[damage][fragment-momentum]") {
    // One synthetic fragment, no blast or skin, stopped by one component. Its
    // measured impact energy independently determines the target's impulse.
    Warhead w;
    w.fragments = 1;
    w.fragmentKg = .01;
    w.fragmentAreaM2 = .0001;
    w.speedMs = 1000;
    w.zones = {{0, 0, 1}};
    Airframe frame;
    Airframe::Component component;
    component.centre = {10, 0, 0};
    component.half = {.5, .5, .5};
    frame.components.push_back(component);
    Target target;
    target.bodyToEcef = {V3{1, 0, 0}, V3{0, 1, 0}, V3{0, 0, 1}};
    Burst burst;
    burst.axis = {1, 0, 0};
    fsim::Rng rng(1);
    const auto hit = assess(w, burst, frame, target, rng);
    REQUIRE(hit.components.size() == 1);
    REQUIRE(hit.fragmentsStriking == 1);
    const double expectedMomentum = std::sqrt(2 * w.fragmentKg * hit.components.front().energyJ);
    REQUIRE(expectedMomentum < w.fragmentKg * w.speedMs);
    CHECK(near(hit.impulseBody.x, expectedMomentum, 1e-12));
    CHECK(near(hit.impulseBody.y, 0, 1e-12));
    CHECK(near(hit.impulseBody.z, 0, 1e-12));
}

namespace {

Warhead straightFragment() {
    Warhead out;
    out.fragments = 1;
    out.fragmentKg = .006;
    out.fragmentAreaM2 = 1.5 * .009 * .009;
    out.speedMs = 300;
    out.zones = {{0, 0, 1}};
    return out;
}

Target stationaryDamageTarget() {
    Target out;
    out.bodyToEcef = {V3{1, 0, 0}, V3{0, 1, 0}, V3{0, 0, 1}};
    out.airDensity = 0;
    return out;
}

std::shared_ptr<Airframe> splitDamageFrame() {
    auto out = std::make_shared<Airframe>();
    out->sections.resize(2);
    out->sections[0].name = "attached";
    auto& removable = out->sections[1];
    removable.name = "removable";
    removable.hasRoot = true;
    removable.skinAreaM2 = 2;
    removable.skinKg = 2 * 2780 * .02;
    return out;
}

std::vector<double> impactValues(const Impact& impact) {
    std::vector<double> out;
    auto vector = [&](V3 v) { out.insert(out.end(), {v.x, v.y, v.z}); };
    out.insert(out.end(), {impact.missM, static_cast<double>(impact.fragmentsThrown), static_cast<double>(impact.fragmentsStriking)});
    vector(impact.fromBody), vector(impact.impulseBody), vector(impact.angularImpulseBody);
    out.push_back(static_cast<double>(impact.components.size()));
    for (const auto& c : impact.components)
        out.insert(out.end(), {static_cast<double>(c.component), static_cast<double>(c.hits), c.energyJ, c.fastestMs, c.areaM2});
    out.push_back(static_cast<double>(impact.panels.size()));
    for (const auto& p : impact.panels) {
        out.insert(out.end(), {static_cast<double>(p.panel), static_cast<double>(p.hits), static_cast<double>(p.perforations),
                              p.holeAreaM2, p.overpressurePa, p.impulsePas});
        vector(p.momentumNs);
    }
    return out;
}

} // namespace

TEST_CASE("a fragment born inside a body box crosses its outward skin", "[damage][fragment-boundary]") {
    // Synthetic closed box, one straight fragment, no blast or air drag. The
    // fragment stops at the positive-X wall and transfers its initial momentum.
    const auto w = straightFragment();
    REQUIRE(thorResidual(.02, w.fragmentAreaM2, w.fragmentKg, w.speedMs, 0) == 0);
    Airframe frame;
    frame.sections.resize(1);
    frame.sections[0].skinAreaM2 = 24;
    frame.sections[0].skinKg = 24 * 2780 * .02;
    Airframe::Panel box;
    box.section = 0;
    box.surface = false;
    box.half = {1, 1, 1};
    box.areaM2 = 24;
    frame.panels.push_back(box);
    Burst burst;
    burst.axis = {1, 0, 0};
    SECTION("born inside the box") {}
    SECTION("born on the inward wall") { burst.at = {-1, 0, 0}; }
    SECTION("born on the outward wall") { burst.at = {1, 0, 0}; }
    fsim::Rng rng(19);
    const auto hit = assess(w, burst, frame, stationaryDamageTarget(), rng);
    CHECK(hit.components.empty());
    CHECK(hit.fragmentsStriking == 1);
    REQUIRE(hit.panels.size() == 1);
    CHECK(hit.panels[0].panel == 0);
    CHECK(hit.panels[0].hits == 1);
    CHECK(hit.panels[0].perforations == 0);
    CHECK(near(hit.impulseBody.x, w.fragmentKg * w.speedMs, 1e-12));
    CHECK(near(hit.impulseBody.y, 0, 1e-12));
    CHECK(near(hit.impulseBody.z, 0, 1e-12));
    CHECK(near(length(hit.angularImpulseBody), 0, 1e-12));
}

TEST_CASE("a nonzero fragment direction cannot be classified as parallel to a box", "[damage][fragment-boundary]") {
    // Exact powers of two: at x=1 this ray has y=dy. The finite box either
    // contains that point or is disjoint; no angular/distance tolerance is needed.
    const double dy=std::ldexp(1.0,-44);
    auto w=straightFragment();
    Airframe frame;
    Airframe::Component component;
    component.centre={1,dy,0};component.half={.25,dy/8,1};
    Burst burst;burst.axis={1,dy,0};
    bool expected=true;
    SECTION("finite shallow crossing is retained") {}
    SECTION("a ray leaving the thin slab before reaching the box misses") {
        component.centre.y=0;expected=false;
    }
    SECTION("a genuinely parallel ray outside the box misses") {burst.axis={1,0,0};expected=false;}
    frame.components.push_back(component);
    fsim::Rng rng(61);
    const auto hit=assess(w,burst,frame,stationaryDamageTarget(),rng);
    CHECK(hit.fragmentsStriking==(expected?1:0));
    REQUIRE(hit.components.size()==(expected?1u:0u));
    if(expected) {
        CHECK(hit.components[0].energyJ==.5*w.fragmentKg*w.speedMs*w.speedMs);
        CHECK(near(hit.impulseBody.y,w.fragmentKg*w.speedMs*dy,1e-25));
    }
}

TEST_CASE("a finite shallow fragment crossing is not parallel to a surface", "[damage][fragment-boundary]") {
    const double dy=std::ldexp(1.0,-32);
    Airframe frame;
    Airframe::Panel panel;
    panel.corners={V3{.75,dy,-1},V3{1.25,dy,-1},V3{1.25,dy,1},V3{.75,dy,1}};
    panel.areaM2=1;frame.panels.push_back(panel);
    Burst burst;burst.axis={1,dy,0};
    bool expected=true;
    SECTION("finite face contains the exact crossing at x one") {}
    SECTION("plane crossing beyond the finite face misses") {burst.axis={1,dy/2,0};expected=false;}
    SECTION("a genuinely parallel off-plane ray misses") {burst.axis={1,0,0};expected=false;}
    SECTION("an unrepresentable crossing cannot enter the finite face through NaN edges") {
        burst.axis={1,std::numeric_limits<double>::denorm_min(),0};expected=false;
    }
    fsim::Rng rng(67);
    const auto w=straightFragment();
    const auto hit=assess(w,burst,frame,stationaryDamageTarget(),rng);
    CHECK(hit.fragmentsStriking==(expected?1:0));
    REQUIRE(hit.panels.size()==(expected?1u:0u));
    if(expected) {
        CHECK(hit.panels[0].hits==1);
        CHECK(hit.panels[0].perforations==0);
        CHECK(near(length(hit.impulseBody),w.fragmentKg*w.speedMs,1e-12));
    }
}

TEST_CASE("a sub-millimetre surface crossing cannot add fragment energy", "[damage][fragment-boundary]") {
    // The burst is 0.5 mm in front of a synthetic stopping skin. Air drag can
    // remove energy before impact; it cannot increase stopped momentum above
    // the fragment's initial momentum, regardless of a skin-event offset.
    const auto w = straightFragment();
    REQUIRE(thorResidual(.02, w.fragmentAreaM2, w.fragmentKg, w.speedMs, 0) == 0);
    Airframe frame;
    frame.sections.resize(1);
    frame.sections[0].skinAreaM2 = 4;
    frame.sections[0].skinKg = 4 * 2780 * .02;
    Airframe::Panel skin;
    skin.section = 0;
    skin.corners = {V3{0, -1, -1}, V3{0, 1, -1}, V3{0, 1, 1}, V3{0, -1, 1}};
    skin.areaM2 = 4;
    frame.panels.push_back(skin);
    auto target = stationaryDamageTarget();
    target.airDensity = 1.225;
    Burst burst;
    burst.at = {-.0005, 0, 0};
    burst.axis = {1, 0, 0};
    SECTION("born half a millimetre before the surface") {}
    SECTION("born on the surface") { burst.at = {}; }
    fsim::Rng rng(23);
    const auto hit = assess(w, burst, frame, target, rng);
    REQUIRE(hit.panels.size() == 1);
    CHECK(hit.panels[0].hits == 1);
    CHECK(hit.panels[0].perforations == 0);
    CHECK(hit.fragmentsStriking == 1);
    const double stoppedMomentum = length(hit.impulseBody);
    const double initialMomentum = w.fragmentKg * w.speedMs;
    const double stoppedEnergy = stoppedMomentum * stoppedMomentum / (2 * w.fragmentKg);
    const double initialEnergy = .5 * w.fragmentKg * w.speedMs * w.speedMs;
    CHECK(stoppedMomentum <= initialMomentum * (1 + 1e-12));
    CHECK(stoppedEnergy <= initialEnergy * (1 + 1e-12));
    CHECK(near(length(hit.impulseBody - hit.panels[0].momentumNs), 0, 1e-12));
}

TEST_CASE("a fragment inside a box uses the outward face normal", "[damage][fragment-boundary]") {
    // This ray's backward intersection is the negative-X wall, but its forward
    // exit is the positive-Y wall. Only the exit's obliquity belongs to the hit.
    auto w = straightFragment();
    w.speedMs = 1800;
    Airframe frame;
    Airframe::Panel box;
    box.surface = false;
    box.half = {1, 1, 1};
    box.areaM2 = 24;
    frame.panels.push_back(box);
    Burst burst;
    burst.at = {-.9, 0, 0};
    burst.axis = {.6, .8, 0};
    fsim::Rng rng(29);
    const auto hit = assess(w, burst, frame, stationaryDamageTarget(), rng);
    const double residual = thorResidual(.002, w.fragmentAreaM2, w.fragmentKg, w.speedMs, std::acos(.8));
    REQUIRE(residual > 0);
    REQUIRE(residual < w.speedMs);
    REQUIRE(hit.panels.size() == 1);
    CHECK(hit.panels[0].hits == 1);
    CHECK(hit.panels[0].perforations == 1);
    const double expectedMomentum = w.fragmentKg * (w.speedMs - residual);
    CHECK(near(hit.impulseBody.x, .6 * expectedMomentum, 1e-12));
    CHECK(near(hit.impulseBody.y, .8 * expectedMomentum, 1e-12));
    CHECK(near(hit.impulseBody.z, 0, 1e-12));
}

TEST_CASE("an internal component precedes the skin a fragment would exit", "[damage][fragment-boundary]") {
    // A fragment born inside a closed box meets this component before the
    // positive-X wall. The later stopping wall must not be moved ahead of it.
    const auto w = straightFragment();
    Airframe frame;
    frame.sections.resize(1);
    frame.sections[0].skinAreaM2 = 24;
    frame.sections[0].skinKg = 24 * 2780 * .02;
    Airframe::Panel box;
    box.section = 0;
    box.surface = false;
    box.half = {1, 1, 1};
    box.areaM2 = 24;
    frame.panels.push_back(box);
    Airframe::Component component;
    component.section = 0;
    component.centre = {.5, 0, 0};
    component.half = {.1, .1, .1};
    frame.components.push_back(component);
    Burst burst;
    burst.axis = {1, 0, 0};
    fsim::Rng rng(31);
    const auto hit = assess(w, burst, frame, stationaryDamageTarget(), rng);
    REQUIRE(hit.components.size() == 1);
    CHECK(hit.components[0].component == 0);
    CHECK(hit.components[0].energyJ == .5 * w.fragmentKg * w.speedMs * w.speedMs);
    CHECK(hit.panels.empty());
    CHECK(near(hit.impulseBody.x, w.fragmentKg * w.speedMs, 1e-12));
}

TEST_CASE("fragment shielding follows actual ray order instead of a nearby future skin", "[damage][fragment-order]") {
    // Analytic fixture: the component begins at x=.9, while the plate is at
    // x=1.5. The plate cannot shield an earlier interception, even though it
    // lies less than a metre away. Moving the plate upstream must shield it.
    const auto w=straightFragment();
    Airframe frame;frame.sections.resize(1);
    frame.sections[0].skinAreaM2=4;
    frame.sections[0].skinKg=4*2780*.02;
    Airframe::Component component;component.section=0;
    component.centre={1,0,0};component.half={.1,.1,.1};
    frame.components.push_back(component);
    Airframe::Panel plate;plate.section=0;plate.areaM2=4;
    double plateX=1.5;
    bool shield=false;
    SECTION("a later plate cannot stop a fragment before its first component") {}
    SECTION("a crossed upstream plate still stops the fragment") {plateX=.5;shield=true;}
    plate.corners={V3{plateX,-1,-1},V3{plateX,1,-1},V3{plateX,1,1},V3{plateX,-1,1}};
    frame.panels.push_back(plate);
    REQUIRE(thorResidual(.02,w.fragmentAreaM2,w.fragmentKg,w.speedMs,0)==0);
    Burst burst;burst.axis={1,0,0};fsim::Rng rng(53);
    const auto hit=assess(w,burst,frame,stationaryDamageTarget(),rng);
    REQUIRE(hit.fragmentsStriking==1);
    if(shield) {
        CHECK(hit.components.empty());REQUIRE(hit.panels.size()==1);
        CHECK(hit.panels[0].hits==1);CHECK(hit.panels[0].perforations==0);
    } else {
        REQUIRE(hit.components.size()==1);CHECK(hit.components[0].component==0);
        CHECK(hit.components[0].energyJ==.5*w.fragmentKg*w.speedMs*w.speedMs);
        CHECK(hit.panels.empty());
    }
    CHECK(near(hit.impulseBody.x,w.fragmentKg*w.speedMs,1e-12));
    CHECK(near(length(hit.angularImpulseBody),0,1e-12));
}

TEST_CASE("nested damage boxes retain initial fragment containment", "[damage][fragment-boundary][fragment-occupancy]") {
    // Both closed boxes contain the burst and component. Neither future exit
    // may be advanced ahead of the component just because the count is even.
    const auto w = straightFragment();
    Airframe frame;
    frame.sections.resize(1);
    frame.sections[0].skinAreaM2 = 120;
    frame.sections[0].skinKg = 120 * 2780 * .02;
    Airframe::Panel box;
    box.section = 0;
    box.surface = false;
    box.half = {1, 1, 1};
    box.areaM2 = 24;
    frame.panels.push_back(box);
    box.half = {2, 2, 2};
    box.areaM2 = 96;
    frame.panels.push_back(box);
    Airframe::Component component;
    component.section = 0;
    component.centre = {.5, 0, 0};
    component.half = {.1, .1, .1};
    frame.components.push_back(component);
    Burst burst;
    burst.axis = {1, 0, 0};
    fsim::Rng rng(37);
    const auto hit = assess(w, burst, frame, stationaryDamageTarget(), rng);
    REQUIRE(hit.components.size() == 1);
    CHECK(hit.components[0].component == 0);
    CHECK(hit.components[0].energyJ == .5 * w.fragmentKg * w.speedMs * w.speedMs);
    CHECK(hit.panels.empty());
    CHECK(near(hit.impulseBody.x, w.fragmentKg * w.speedMs, 1e-12));
}

TEST_CASE("leaving one box preserves fragment containment in another", "[damage][fragment-boundary][fragment-occupancy]") {
    // The inner skin perforates at x=1. A component at x=1.5 is still inside
    // the outer box, whose stopping skin at x=2 must remain behind the hit.
    const auto w = straightFragment();
    const double residual = thorResidual(.0005, w.fragmentAreaM2, w.fragmentKg, w.speedMs, 0);
    REQUIRE(residual > 0);
    REQUIRE(residual < w.speedMs);
    REQUIRE(thorResidual(.02, w.fragmentAreaM2, w.fragmentKg, residual, 0) == 0);
    Airframe frame;
    frame.sections.resize(2);
    frame.sections[0].skinAreaM2 = 24;
    frame.sections[0].skinKg = 24 * 2780 * .0005;
    frame.sections[1].skinAreaM2 = 96;
    frame.sections[1].skinKg = 96 * 2780 * .02;
    Airframe::Panel box;
    box.section = 0;
    box.surface = false;
    box.half = {1, 1, 1};
    box.areaM2 = 24;
    frame.panels.push_back(box);
    box.section = 1;
    box.half = {2, 2, 2};
    box.areaM2 = 96;
    frame.panels.push_back(box);
    Airframe::Component component;
    component.section = 1;
    component.centre = {1.5, 0, 0};
    component.half = {.1, .1, .1};
    frame.components.push_back(component);
    Burst burst;
    burst.axis = {1, 0, 0};
    fsim::Rng rng(41);
    const auto hit = assess(w, burst, frame, stationaryDamageTarget(), rng);
    REQUIRE(hit.components.size() == 1);
    CHECK(hit.components[0].component == 0);
    CHECK(near(hit.components[0].energyJ, .5 * w.fragmentKg * residual * residual, 1e-10));
    REQUIRE(hit.panels.size() == 1);
    CHECK(hit.panels[0].panel == 0);
    CHECK(hit.panels[0].hits == 1);
    CHECK(hit.panels[0].perforations == 1);
    CHECK(near(hit.impulseBody.x, w.fragmentKg * w.speedMs, 1e-12));
}

TEST_CASE("attached damage geometry removes shields and components without reindexing the source", "[damage][attached-geometry]") {
    auto frame = splitDamageFrame();
    Airframe::Component missed;
    missed.section = 0;
    missed.centre = {10, 10, 0};
    missed.half = {.5, .5, .5};
    frame->components.push_back(missed);
    auto removed = missed;
    removed.section = 1;
    removed.centre = {5, 0, 0};
    frame->components.push_back(removed);
    auto behind = missed;
    behind.centre = {10, 0, 0};
    frame->components.push_back(behind);
    SECTION("a removed skin no longer shields the attached component") {
        frame->components[1].centre.y = 10;
        Airframe::Panel skin;
        skin.section = 1;
        skin.surface = false;
        skin.centre = {5, 0, 0};
        skin.half = {.1, .5, .5};
        skin.areaM2 = 2;
        frame->panels.push_back(skin);
    }
    SECTION("a removed component no longer absorbs a fragment before the attached component") {}
    VehicleDamage first(frame), untouched(frame);
    const auto w = straightFragment();
    REQUIRE(thorResidual(.02, w.fragmentAreaM2, w.fragmentKg, w.speedMs, 0) == 0);
    Burst burst;
    burst.axis = {1, 0, 0};
    const auto target = stationaryDamageTarget();
    fsim::Rng fullRng(11), viewRng(11), unchangedRng(11);
    const auto full = assess(w, burst, *frame, target, fullRng);
    const auto intact = assess(w, burst, first.attachedGeometry(), target, viewRng);
    CHECK(impactValues(full) == impactValues(intact));
    CHECK(fullRng.state() == viewRng.state());
    REQUIRE(first.separate(1));
    const auto active = first.attachedGeometry();
    CHECK(&active.airframe() == frame.get());
    CHECK(active.panelIndices().empty());
    REQUIRE(active.componentIndices().size() == 2);
    CHECK(active.componentIndices()[0] == 0);
    CHECK(active.componentIndices()[1] == 2);
    fsim::Rng activeRng(11);
    const auto exposed = assess(w, burst, active, target, activeRng);
    REQUIRE(exposed.components.size() == 1);
    CHECK(exposed.components[0].component == 2);
    CHECK(exposed.components[0].energyJ == .5 * w.fragmentKg * w.speedMs * w.speedMs);
    CHECK(exposed.panels.empty());
    CHECK(exposed.fragmentsStriking == 1);
    CHECK(activeRng.state() == fullRng.state());
    CHECK(impactValues(assess(w, burst, untouched.attachedGeometry(), target, unchangedRng)) == impactValues(full));
    CHECK_FALSE(untouched.sections()[1].separated);
    CHECK(frame->sections[1].hasRoot);
    CHECK(frame->components.size() == 3);
}

TEST_CASE("damage views preserve unsectioned fixtures and exclude invalid positive memberships", "[damage][attached-geometry]") {
    auto frame = std::make_shared<Airframe>();
    Airframe::Component invalid;
    invalid.section = 0; // No section 0 exists.
    invalid.centre = {5, 0, 0};
    invalid.half = {.5, .5, .5};
    frame->components.push_back(invalid);
    auto legacy = invalid;
    legacy.section = -1;
    legacy.centre.x = 10;
    frame->components.push_back(legacy);
    Airframe::Panel badSkin;
    badSkin.section = 7;
    badSkin.surface = false;
    badSkin.centre = {3, 0, 0};
    badSkin.half = {.1, 1, 1};
    badSkin.areaM2 = 10;
    frame->panels.push_back(badSkin);
    VehicleDamage damage(frame);
    const auto view = damage.attachedGeometry();
    CHECK(view.panelIndices().empty());
    REQUIRE(view.componentIndices().size() == 1);
    CHECK(view.componentIndices()[0] == 1);
    auto w = straightFragment();
    w.fragments = 4;
    Burst burst;
    burst.axis = {1, 0, 0};
    fsim::Rng directRng(17), viewRng(17), expected(17);
    const auto direct = assess(w, burst, *frame, stationaryDamageTarget(), directRng);
    const auto held = assess(w, burst, view, stationaryDamageTarget(), viewRng);
    REQUIRE(direct.components.size() == 1);
    CHECK(direct.components[0].component == 1);
    CHECK(impactValues(direct) == impactValues(held));
    CHECK(direct.panels.empty());
    for (int i = 0; i < 3 * w.fragments; ++i) expected.next();
    CHECK(directRng.state() == expected.state());
    CHECK(viewRng.state() == expected.state());
}

TEST_CASE("detached blast panels contribute no parent impulse and retain source panel identities", "[damage][attached-geometry]") {
    auto frame = splitDamageFrame();
    Airframe::Panel panel;
    panel.section = 1;
    panel.surface = false;
    panel.centre = {10, 0, 0};
    panel.half = {.1, 1, 1};
    panel.areaM2 = 8;
    frame->panels.push_back(panel);
    panel.section = 0;
    panel.centre = {0, 10, 0};
    frame->panels.push_back(panel);
    VehicleDamage damage(frame);
    REQUIRE(damage.separate(1));
    Warhead w;
    w.tntKg = 2;
    Burst burst;
    burst.at = {0, 0, 5};
    const auto active = damage.attachedGeometry();
    REQUIRE(active.panelIndices().size() == 1);
    CHECK(active.panelIndices()[0] == 1);
    fsim::Rng rng(23);
    const auto impact = assess(w, burst, active, stationaryDamageTarget(), rng);
    REQUIRE(impact.panels.size() == 1);
    CHECK(impact.panels[0].panel == 1);
    const auto impulse = panelBlastImpulse(frame->panels[1], burst.at, impact.panels[0].impulsePas);
    CHECK(near(length(impact.impulseBody - impulse), 0, 1e-12));
    CHECK(near(length(impact.angularImpulseBody - cross(frame->panels[1].middle(), impulse)), 0, 1e-12));
    frame = splitDamageFrame();
    panel.section = 1;
    frame->panels.push_back(panel);
    VehicleDamage empty(frame);
    REQUIRE(empty.separate(1));
    const auto absent = assess(w, burst, empty.attachedGeometry(), stationaryDamageTarget(), rng);
    CHECK(absent.panels.empty());
    CHECK(absent.components.empty());
    CHECK(length(absent.impulseBody) == 0);
    CHECK(length(absent.angularImpulseBody) == 0);
}

TEST_CASE("one canonical panel blast impulse includes half area and projected surface facing", "[damage][panel-impulse]") {
    Airframe::Panel panel;
    panel.areaM2 = 8;
    panel.corners = {V3{10, -1, -1}, V3{10, 1, -1}, V3{10, 1, 1}, V3{10, -1, 1}};
    CHECK(near(length(panelBlastImpulse(panel, {0, 0, 0}, 100) - V3{400, 0, 0}), 0, 1e-12));
    CHECK(near(length(panelBlastImpulse(panel, {0, 0, 10}, 100) - V3{200, 0, -200}), 0, 1e-12));
    CHECK(length(panelBlastImpulse(panel, {10, 0, 10}, 100)) == 0);
    CHECK(length(panelBlastImpulse(panel, {0, 0, 0}, 0)) == 0);
    panel.surface = false;
    panel.centre = {10, 0, 0};
    CHECK(near(length(panelBlastImpulse(panel, {0, 0, 0}, 100) - V3{400, 0, 0}), 0, 1e-12));
}

TEST_CASE("stale detached and invalid damage hits change no state or random stream", "[damage][attached-geometry]") {
    auto frame = splitDamageFrame();
    Airframe::Component fuel;
    fuel.section = 1;
    fuel.kind = "fuel";
    fuel.tank = 0;
    fuel.fuelKg = 100;
    frame->components.push_back(fuel);
    auto invalid = fuel;
    invalid.section = 42;
    invalid.tank = 1;
    frame->components.push_back(invalid);
    Airframe::Panel panel;
    panel.section = 1;
    panel.areaM2 = 2;
    frame->panels.push_back(panel);
    panel.section = 42;
    frame->panels.push_back(panel);
    VehicleDamage damage(frame);
    REQUIRE(damage.separate(1));
    Impact stale;
    for (const int id : {0, 1, -1, 99}) {
        Impact::ComponentHits hit;
        hit.component = id;
        hit.hits = 5;
        hit.energyJ = 1e9;
        hit.fastestMs = 2000;
        hit.areaM2 = .01;
        stale.components.push_back(hit);
        Impact::PanelHits skin;
        skin.panel = id;
        skin.hits = skin.perforations = 5;
        skin.holeAreaM2 = .1;
        skin.impulsePas = 1e6;
        stale.panels.push_back(skin);
    }
    fsim::Rng rng(29);
    const auto before = rng.state();
    CHECK(damage.apply(stale, 101325, rng).empty());
    CHECK(rng.state() == before);
    CHECK(damage.tanks().empty());
    CHECK_FALSE(damage.panelTorn(0));
    CHECK_FALSE(damage.panelTorn(1));
    CHECK(damage.sections()[1].strength == 1);
    CHECK(damage.sections()[1].holeM2 == 0);
    CHECK(damage.sections()[1].tornM2 == 0);
    CHECK(damage.sections()[1].removed == 0);
}

TEST_CASE("every aircraft's damage data reads", "[damage]") {
    int read = 0;
    for (const auto& d : std::filesystem::directory_iterator(kAircraft)) {
        const auto file = d.path() / (d.path().filename().string() + ".damage.json");
        if (!std::filesystem::is_regular_file(file)) continue;
        std::string error;
        const auto a = readAirframe(file, error);
        INFO(error);
        REQUIRE(a);
        CHECK(a->sections.size() == 5);
        CHECK(!a->panels.empty());
        CHECK(!a->components.empty());
        for (const auto& p : a->panels) CHECK(p.section >= 0);
        ++read;
    }
    CHECK(read >= 30);
}

TEST_CASE("an AIM-120C bursting 6 m to an F-16C's left: its fragments strike the left of it", "[damage]") {
    std::string error;
    const auto f16 = readAirframe(kAircraft / "f16c" / "f16c.damage.json", error);
    REQUIRE(f16);
    const auto w = warheadOf(*card("aim-120c"));
    REQUIRE(w);
    // the target level, flying north (body x north, y east, z down: ECEF here a local frame), at 220 m/s; the missile
    // from behind and below at 800 m/s, bursting 6 m to its left by its wing
    Target t;
    t.bodyToEcef = {V3{1, 0, 0}, V3{0, 1, 0}, V3{0, 0, 1}};
    t.velocity = {220.0, 0.0, 0.0};
    t.airPressurePa = 47181.0, t.airTemperatureK = 249.2, t.airDensity = 0.66;
    Burst b;
    b.at = {-1.0, -6.0, 0.5};
    b.velocity = {800.0, 50.0, -40.0};
    b.axis = b.velocity;
    fsim::Rng rng(7);
    const Impact hit = assess(*w, b, *f16, t, rng);
    CHECK(near(hit.missM, std::sqrt(1.0 + 36.0 + 0.25), 1e-9));
    if (std::getenv("FSIM_DAMAGE_SHOW")) {
        std::printf("AIM-120C: %d fragments of %.1f g at %.0f m/s, %.2f m long, case %.1f mm; %d strike\n", w->fragments,
                    1000.0 * w->fragmentKg, w->speedMs, w->lengthM, 1000.0 * w->caseThicknessM, hit.fragmentsStriking);
        for (const auto& c : hit.components)
            std::printf("  %-28s %3d hits %8.0f J, fastest %4.0f m/s\n", f16->components[static_cast<std::size_t>(c.component)].name.c_str(),
                        c.hits, c.energyJ, c.fastestMs);
        for (const auto& p : hit.panels)
            if (p.hits || p.impulsePas > 100.0)
                std::printf("  panel %3d %-10s %3d hits %3d through, hole %5.0f cm2, %6.0f kPa, %5.0f Pa s\n", p.panel,
                            f16->sections[static_cast<std::size_t>(f16->panels[static_cast<std::size_t>(p.panel)].section)].name.c_str(),
                            p.hits, p.perforations, 1e4 * p.holeAreaM2, 1e-3 * p.overpressurePa, p.impulsePas);
        std::printf("  impulse %.0f %.0f %.0f N s\n", hit.impulseBody.x, hit.impulseBody.y, hit.impulseBody.z);
    }
    CHECK(hit.fragmentsStriking > 20);
    CHECK(hit.fragmentsStriking < hit.fragmentsThrown);
    int left = 0, right = 0;
    for (const auto& p : hit.panels) {
        const auto& s = f16->sections[static_cast<std::size_t>(f16->panels[static_cast<std::size_t>(p.panel)].section)].name;
        if (s == "left wing") left += p.hits;
        if (s == "right wing") right += p.hits;
    }
    INFO("left wing " << left << " hits, right " << right);
    CHECK(left > right);
    CHECK(!hit.components.empty());
    CHECK(hit.impulseBody.y > 0.0); // (pushed away from the burst: to the right)
    // the same stream, the same hits
    fsim::Rng again(7);
    const Impact twice = assess(*w, b, *f16, t, again);
    CHECK(twice.fragmentsStriking == hit.fragmentsStriking);
    CHECK(twice.components.size() == hit.components.size());
    // far off, nothing strikes and the blast is weak
    Burst far = b;
    far.at = {0.0, -300.0, 0.0};
    fsim::Rng rng3(7);
    const Impact miss = assess(*w, far, *f16, t, rng3);
    CHECK(miss.components.empty());
}

TEST_CASE("a hit aircraft's structure: an intact wing fails at its ultimate load, a weakened one sooner", "[damage]") {
    std::string error;
    auto f16 = readAirframe(kAircraft / "f16c" / "f16c.damage.json", error);
    REQUIRE(f16);
    const auto af = std::make_shared<const Airframe>(*f16);
    const double g = 9.80665, W = af->designMassKg;
    // flown level, pulling n at its design mass, its tanks empty (the case its root is sized by)
    auto pull = [&](double n) {
        Flight f;
        f.aeroForceN = {0.0, 0.0, -n * W * g};
        f.specificForceMs2 = {0.0, 0.0, -n * g};
        f.tankKg.assign(8, 0.0);
        return f;
    };
    const int left = af->section("left wing");
    VehicleDamage intact(af);
    const double nu = af->nUltimate[1];
    CHECK(near(std::abs(intact.bending(left, pull(nu))[0]), af->sections[static_cast<std::size_t>(left)].strengthNm[0],
               2e-3 * af->sections[static_cast<std::size_t>(left)].strengthNm[0]));
    CHECK(intact.loads(pull(0.99 * nu)).empty());
    VehicleDamage pulled(af);
    const auto broke = pulled.loads(pull(1.02 * nu));
    REQUIRE(!broke.empty());
    CHECK(broke.front().kind == Change::Kind::Separated);
    // its torn panels' face: a blast at a thin skin tears it from some 600 Pa s (4 mm of 2024-T3)
    CHECK(tearImpulse(0.004) > 400.0);
    CHECK(tearImpulse(0.004) < 900.0);
    // a 10 cm2 hole in a tank at 1 g and 30 kPa of dynamic pressure: some kilograms a second
    VehicleDamage holed(af);
    Impact hit;
    for (std::size_t c = 0; c < af->components.size(); ++c)
        if (af->components[c].kind == "fuel") {
            Impact::ComponentHits h;
            h.component = static_cast<int>(c), h.hits = 1, h.energyJ = 0.0, h.fastestMs = 500.0, h.areaM2 = 1e-3;
            hit.components.push_back(h);
            break;
        }
    fsim::Rng rng(3);
    holed.apply(hit, 101325.0, rng);
    REQUIRE(holed.tanks().size() == 1);
    const int tank = holed.tanks().begin()->first;
    const double leak = holed.leakKgS(tank, 500.0, 1.0, 30000.0);
    CHECK(leak > 1.0);
    CHECK(leak < 10.0);
    CHECK(holed.leakKgS(tank, 0.0, 1.0, 30000.0) == 0.0);
}

TEST_CASE("an AIM-120C's burst beside an F-16C: what it breaks, deterministic", "[damage]") {
    std::string error;
    auto f16 = readAirframe(kAircraft / "f16c" / "f16c.damage.json", error);
    REQUIRE(f16);
    const auto af = std::make_shared<const Airframe>(*f16);
    const auto w = warheadOf(*card("aim-120c"));
    REQUIRE(w);
    Target t;
    t.bodyToEcef = {V3{1, 0, 0}, V3{0, 1, 0}, V3{0, 0, 1}};
    t.velocity = {220.0, 0.0, 0.0};
    t.airPressurePa = 47181.0, t.airTemperatureK = 249.2, t.airDensity = 0.66;
    Burst b;
    b.velocity = {800.0, 50.0, -40.0};
    b.axis = b.velocity;
    std::vector<std::vector<int>> runs;
    for (int k = 0; k < 2; ++k) {
        b.at = {-1.0, -3.0, 0.5}; // (3 m off its left side)
        fsim::Rng rng(11);
        const Impact hit = assess(*w, b, *af, t, rng);
        VehicleDamage d(af);
        const auto changes = d.apply(hit, t.airPressurePa, rng);
        std::vector<int> kinds;
        for (const auto& c : changes) kinds.push_back(static_cast<int>(c.kind));
        runs.push_back(kinds);
        if (k == 0 && std::getenv("FSIM_DAMAGE_SHOW"))
            for (const auto& c : changes)
                std::printf("change %d component %d tank %d engine %d channel %s panel %d section %d spill %.1f\n", static_cast<int>(c.kind),
                            c.component, c.tank, c.engine, c.channel.c_str(), c.panel, c.section, c.spillKg);
        CHECK(!changes.empty());
    }
    CHECK(runs[0] == runs[1]);
}

namespace {
// a burst `offset` metres from the vehicle's origin (body axes), the missile closing at `closing` m/s (body axes)
fsim::session::Detonation burstBeside(const fsim::sim::VehicleState& s, V3 offset, V3 closing) {
    fsim::session::Detonation d;
    d.store = "aim-120c";
    const double* R = s.rotationBodyToEcef;
    auto toEcef = [&](const V3& b) { return V3{R[0] * b.x + R[1] * b.y + R[2] * b.z, R[3] * b.x + R[4] * b.y + R[5] * b.z, R[6] * b.x + R[7] * b.y + R[8] * b.z}; };
    const V3 off = toEcef(offset), own = toEcef({s.velocityBodyMs[0], s.velocityBodyMs[1], s.velocityBodyMs[2]}), close = toEcef(closing);
    for (int k = 0; k < 3; ++k) {
        d.positionEcef[k] = s.positionEcef[k] + (&off.x)[k];
        d.velocityEcef[k] = (&own.x)[k] + (&close.x)[k];
        d.axisEcef[k] = d.velocityEcef[k];
    }
    return d;
}

struct NativeMomentProbe {
    V3 cg,impulse;
    unsigned sceneCalls=0,legacyCalls=0;
};
// These are explicit test providers, not altered production cards. The scene
// provider exercises the physical kernel; the external provider below supplies
// a known historical wrench to check that its old convention remains intact.
class FixedReferenceMomentFixture : public WarheadAdapter {
public:
    FixedReferenceMomentFixture(Warhead model,std::shared_ptr<NativeMomentProbe> probe)
        :model_(std::move(model)),probe_(std::move(probe)){model_.tntKg=0;}
    std::string_view implementationId() const noexcept override{return "test.fixed_reference_moment";}
    const Warhead& model() const noexcept override{return model_;}
    Impact assess(const Burst&,const AttachedGeometryView&,const Target&,fsim::Rng&) const override {
        ++probe_->legacyCalls;Impact impact;impact.missM=0;
        impact.impulseBody=probe_->impulse;impact.angularImpulseBody=cross(probe_->cg,probe_->impulse);return impact;
    }
protected:
    Warhead model_;
    std::shared_ptr<NativeMomentProbe> probe_;
};
class MovingOriginMomentFixture final : public FixedReferenceMomentFixture,public WarheadSceneExecutionV2 {
public:
    using FixedReferenceMomentFixture::FixedReferenceMomentFixture;
    static constexpr double massKg=1e-6,relativeSpeedMs=1000;
    FragmentFieldV1 emitScene(const Burst& burst,fsim::Rng&) const override {
        ++probe_->sceneCalls;FragmentFieldV1 field;field.valid=true;field.caseKg=massKg;field.burst=burst;
        field.emittedCount=1;field.applyBlast=false;
        FragmentFieldV1::Projectile p;p.at=burst.at;p.velocity=burst.velocity;p.massKg=massKg;
        p.areaM2=1e-8;p.cd=1.3;p.sourceOrdinalV1=1;field.projectiles.push_back(p);return field;
    }
};
void nativeMomentApplication(unsigned skip,bool turning,bool scene) {
    using namespace fsim::ms::components;
    CAPTURE(skip,turning,scene);
    auto settings=fsim::modes::options("native-moving-origin-moment");settings.frameSkip=static_cast<int>(skip);
    fsim::session::World world(settings);auto probe=std::make_shared<NativeMomentProbe>();
    auto registry=builtInProvidersV1();const auto original=registry.warhead;std::string error;
    REQUIRE(registry.warhead.registerFactory("equipment.aim-120c-7.warhead","native.warhead.finite_zoned_case.v2",
        [original,probe,scene](const auto& descriptor,const auto& context,auto& message)->std::unique_ptr<WarheadAdapter>{
            auto inner=original.create(descriptor,context,message);if(!inner)return {};
            if(scene)return std::make_unique<MovingOriginMomentFixture>(inner->model(),probe);
            return std::make_unique<FixedReferenceMomentFixture>(inner->model(),probe);
        },true,error));
    REQUIRE(world.setPayloadProvidersV1(registry,error));
    fsim::session::VehicleSpec spec;spec.name="moment-victim";spec.type="jsbsim:f16c";
    spec.initial.altitudeMslM=6000;spec.initial.headingDeg=90;spec.initial.airspeedTrueMs=220;
    if(turning){spec.initial.bodyRatesRadS[0]=.02;spec.initial.bodyRatesRadS[1]=-.015;spec.initial.bodyRatesRadS[2]=.01;}
    const auto id=world.createVehicle(spec);REQUIRE(id);
    auto* native=dynamic_cast<fsim::sim::JsbsimModel*>(world.model(id));REQUIRE(native);
    const auto captured=*world.vehicleState(id);double capturedCg[3];REQUIRE(native->cgFromOriginBodyM(capturedCg));
    probe->cg={capturedCg[0],capturedCg[1],capturedCg[2]};
    const auto frame=readAirframe(kAircraft/"f16c/f16c.damage.json",error);REQUIRE(frame);
    const auto component=std::find_if(frame->components.begin(),frame->components.end(),[](const auto& item){
        // This fixture needs a solid absorber at birth: a real liquid has a
        // finite transit time and is covered by the independent fluid suite.
        return item.kind=="crew";
    });
    REQUIRE(component!=frame->components.end());
    const V3 arm=component->centre-probe->cg,ray=normalized(arm);
    const auto rotate=[&](V3 v){const auto* r=captured.rotationBodyToEcef;
        return V3{r[0]*v.x+r[1]*v.y+r[2]*v.z,r[3]*v.x+r[4]*v.y+r[5]*v.z,r[6]*v.x+r[7]*v.y+r[8]*v.z};};
    const V3 capturedAt{captured.positionEcef[0],captured.positionEcef[1],captured.positionEcef[2]};
    const V3 at=capturedAt+rotate(arm);
    const V3 omega{captured.angularRateBodyRadS[0],captured.angularRateBodyRadS[1],captured.angularRateBodyRadS[2]};
    if(turning)REQUIRE(length(omega)>0);
    const V3 velocity=rotate(V3{captured.velocityBodyMs[0],captured.velocityBodyMs[1],captured.velocityBodyMs[2]}+
        cross(omega,arm)+ray*MovingOriginMomentFixture::relativeSpeedMs);
    probe->impulse=ray*(MovingOriginMomentFixture::massKg*MovingOriginMomentFixture::relativeSpeedMs);
    fsim::session::Detonation burst;burst.store="aim-120c";burst.stepFraction=0;
    const V3 axis=rotate(ray);
    for(int i=0;i<3;++i){burst.positionEcef[i]=(&at.x)[i];burst.velocityEcef[i]=(&velocity.x)[i];burst.axisEcef[i]=(&axis.x)[i];}
    world.step();
    const auto before=native->rigid();double currentCg[3];REQUIRE(native->cgFromOriginBodyM(currentCg));
    fsim::sim::RigidMassPropertiesV1 mass;REQUIRE(native->rigidMassPropertiesV1(mass));
    double minimumInertia=std::numeric_limits<double>::infinity(),rotationChangeSquared=0;
    for(unsigned row=0;row<3;++row) {
        double lower=mass.inertiaBodyKgM2[3*row+row];
        for(unsigned col=0;col<3;++col)if(col!=row)lower-=std::abs(mass.inertiaBodyKgM2[3*row+col]);
        minimumInertia=std::min(minimumInertia,lower);
    }
    REQUIRE(minimumInertia>0);
    for(unsigned i=0;i<9;++i){const double change=before.bodyToEcef[i]-captured.rotationBodyToEcef[i];rotationChangeSquared+=change*change;}
    const double cgChange=length(V3{currentCg[0],currentCg[1],currentCg[2]}-probe->cg);
    const double lever=length(arm)+length(probe->cg);
    const double epsilon=std::numeric_limits<double>::epsilon();
    const double positionRoundoff=128*epsilon*std::max({1.,length(capturedAt),length(at)});
    // The source is strictly INSIDE the actual component at birth. Its first
    // absorption has flight time zero: gravity/drag cannot contribute impulse.
    // The independent r x J is zero about captured CG. Bound the current
    // frozen-pose/CG approximation using observed rotation/CG change, the known
    // impulse, and a Gershgorin lower bound on the actual full native inertia.
    // The bound is fixed BEFORE reading any post-damage rates or test outcome.
    constexpr double firstFlightS=0,gravityMs2=9.80665;
    const double materialLeverBound=cgChange+lever*std::sqrt(rotationChangeSquared)+positionRoundoff+
        .5*gravityMs2*firstFlightS*firstFlightS;
    REQUIRE(materialLeverBound<.5*std::min({component->half.x,component->half.y,component->half.z}));
    const double rateRoundoff=128*epsilon*(1+length(V3{before.ratesBody[0],before.ratesBody[1],before.ratesBody[2]}));
    const double rateBound=length(probe->impulse)*materialLeverBound/minimumInertia+rateRoundoff;
    const auto currentBody=[&](V3 v){const auto* r=before.bodyToEcef;
        return V3{r[0]*v.x+r[3]*v.y+r[6]*v.z,r[1]*v.x+r[4]*v.y+r[7]*v.z,r[2]*v.x+r[5]*v.y+r[8]*v.z};};
    const V3 currentAt{before.cgEcef[0],before.cgEcef[1],before.cgEcef[2]};
    const V3 expectedLegacy=currentBody(cross(capturedAt-currentAt,rotate(probe->impulse)));
    const double legacyMomentBound=length(probe->impulse)*(cgChange+positionRoundoff)+
        128*epsilon*std::max(1.,length(expectedLegacy));
    const auto events=world.detonate(burst);
    REQUIRE(world.damage(id));
    const auto actualComponent=std::find_if(world.damage(id)->airframe().components.begin(),world.damage(id)->airframe().components.end(),
        [&](const auto& item){return item.name==component->name;});
    REQUIRE(actualComponent!=world.damage(id)->airframe().components.end());
    REQUIRE(length(actualComponent->centre-component->centre)<positionRoundoff);
    const auto after=native->rigid();
    REQUIRE(after.massKg==before.massKg);REQUIRE(world.vehicleIds().size()==1);
    const V3 delta{after.ratesBody[0]-before.ratesBody[0],after.ratesBody[1]-before.ratesBody[1],after.ratesBody[2]-before.ratesBody[2]};
    INFO("native spin increment="<<length(delta)<<" independent bound="<<rateBound);
    if(scene) {
        REQUIRE(probe->sceneCalls==1);CHECK(probe->legacyCalls==0);
        REQUIRE(std::any_of(events.begin(),events.end(),[](const auto& event){
            return event.kind==static_cast<std::uint8_t>(fsim::ipc::EngagementKind::Hit)&&event.index==1;
        }));
        CHECK(length(delta)<=rateBound);
    } else {
        REQUIRE(probe->legacyCalls==1);CHECK(probe->sceneCalls==0);
        V3 measured;for(unsigned row=0;row<3;++row)for(unsigned col=0;col<3;++col)
            (&measured.x)[row]+=mass.inertiaBodyKgM2[3*row+col]*(&delta.x)[col];
        REQUIRE(length(expectedLegacy)>10*legacyMomentBound);
        CHECK(length(measured-expectedLegacy)<=legacyMomentBound);
    }
}
} // namespace

TEST_CASE("native shared scene spin respects the moving origin while external fixed moments retain transport", "[damage][world][scene-effects][scene-native-moving-origin]") {
    for(const unsigned skip:{1u,4u})for(const bool turning:{false,true})for(const bool scene:{true,false})
        nativeMomentApplication(skip,turning,scene);
}

TEST_CASE("an AIM-120C beside a MiG-29A in the world: what it breaks, flown on, deterministic", "[damage][world]") {
    std::vector<std::vector<int>> kinds;
    for (int run = 0; run < 2; ++run) {
        fsim::session::World w(fsim::modes::options("damage"));
        const auto id = fsim::modes::wing(w, "mig29a", 6000.0, 220.0);
        w.step(fsim::modes::stepsFor(w, 3.0));
        const auto& s = *w.vehicleState(id);
        // 4 m off its left side, overtaking from behind and below at 600 m/s
        const auto events = w.detonate(burstBeside(s, {-1.0, -4.0, 0.5}, {600.0, 80.0, -30.0}));
        REQUIRE(!events.empty());
        CHECK(events.front().kind == static_cast<std::uint8_t>(fsim::ipc::EngagementKind::Detonation));
        CHECK(events.front().target != 0); // (its target: the MiG)
        const auto* dmg = w.damage(id);
        REQUIRE(dmg);
        std::vector<double> before;
        auto fuel = [&] {
            std::vector<double> out;
            for (int t = 0; t < 3; ++t) out.push_back(w.model(id)->property("propulsion/tank[" + std::to_string(t) + "]/contents-lbs").get());
            return out;
        };
        before = fuel();
        w.step(fsim::modes::stepsFor(w, 5.0));
        const auto later = w.takeEngagementEvents();
        const auto after = fuel();
        std::vector<int> k;
        for (const auto& e : later) k.push_back(e.kind);
        kinds.push_back(k);
        if (run == 0 && std::getenv("FSIM_DAMAGE_SHOW")) {
            for (const auto& e : later)
                std::printf("event %d %-24s t %.2f index %u spill %.1f rate %.2f\n", e.kind, e.name, e.simTime, e.index, e.amountKg, e.rateKgS);
            for (std::size_t t = 0; t < before.size(); ++t) std::printf("tank %zu: %.0f -> %.0f lbs\n", t, before[t], after[t]);
            const auto& v = *w.vehicleState(id);
            std::printf("after 5 s: alt %.0f m, roll %.0f, pitch %.0f deg, tas %.0f m/s\n", v.altitudeMslM, v.eulerRad[0] * 57.3, v.eulerRad[1] * 57.3,
                        v.airspeedTrueMs);
        }
        // its holed tanks lost fuel faster than its engines burn it
        bool leaked = false;
        for (const auto& [tank, state] : dmg->tanks())
            if (tank >= 0 && tank < 3 && state.holeM2 > 0.0 && after[static_cast<std::size_t>(tank)] < before[static_cast<std::size_t>(tank)]) leaked = true;
        CHECK((dmg->tanks().empty() || leaked));
        CHECK(std::isfinite(w.vehicleState(id)->altitudeMslM));
    }
    CHECK(kinds[0] == kinds[1]);
}

TEST_CASE("a MiG-29A's left wing parts: it flies on as a JSBSim piece of its own, the aircraft rolls", "[damage][world]") {
    fsim::session::World w(fsim::modes::options("piece"));
    const auto id = fsim::modes::wing(w, "mig29a", 6000.0, 220.0);
    w.step(fsim::modes::stepsFor(w, 2.0));
    const double before = w.model(id)->property("inertia/weight-lbs").get();
    REQUIRE(w.separateSection(id, "left wing"));
    CHECK_FALSE(w.separateSection(id, "left wing")); // (gone already)
    std::uint32_t piece = 0;
    for (const auto v : w.vehicleIds())
        if (w.info(v)->role == fsim::ms::Role::Debris) piece = v;
    REQUIRE(piece != 0);
    CHECK(w.info(piece)->type.rfind("debris:mig29a:left wing:", 0) == 0);
    const auto& p0 = *w.vehicleState(piece);
    const auto& a0 = *w.vehicleState(id);
    const double alt0 = p0.altitudeMslM;
    // it leaves where its centre was, as fast as the aircraft
    CHECK(std::abs(p0.airspeedTrueMs - a0.airspeedTrueMs) < 15.0);
    // the aircraft lighter by the wing's structure
    std::string error;
    const auto af = readAirframe(kAircraft / "mig29a" / "mig29a.damage.json", error);
    REQUIRE(af);
    const double wing = af->sections[static_cast<std::size_t>(af->section("left wing"))].massKg;
    w.step(1);
    CHECK(std::abs((before - w.model(id)->property("inertia/weight-lbs").get()) * 0.45359237 - wing) < 0.05 * wing + 20.0);
    // its first seconds: unstable in pitch (its centre of mass behind its centre of pressure) it pitches up, its
    // lift lifting it, and tumbles, its broadside drag taking its speed; then it falls at a tumbling plate's speed
    w.step(fsim::modes::stepsFor(w, 8.0));
    const auto& p1 = *w.vehicleState(piece);
    const auto& a1 = *w.vehicleState(id);
    CHECK(p1.airspeedTrueMs < 0.6 * a1.airspeedTrueMs); // (its drag a tumbling plate's)
    CHECK(std::abs(a1.eulerRad[0]) > 0.5);              // (one wing gone: the aircraft rolls off)
    const double rollAt8 = a1.eulerRad[0] * 57.2958, climbed = p1.altitudeMslM - alt0;
    w.step(fsim::modes::stepsFor(w, 22.0));
    const auto& p2 = *w.vehicleState(piece);
    if (std::getenv("FSIM_DAMAGE_SHOW"))
        std::printf("the piece: up %.0f m by 8 s (the aircraft rolled to %.0f deg), %.0f m below its start by 30 s at %.0f m/s\n", climbed,
                    rollAt8, alt0 - p2.altitudeMslM, p2.airspeedTrueMs);
    CHECK(p2.altitudeMslM < alt0 - 300.0);
    CHECK(p2.airspeedTrueMs < 120.0);
    const auto events = w.takeEngagementEvents();
    bool separated = false;
    for (const auto& e : events) separated = separated || e.kind == static_cast<std::uint8_t>(fsim::ipc::EngagementKind::Separated);
    CHECK(separated);
}

TEST_CASE("what a JSBSim piece costs a step", "[.][bench][damage]") {
    // two worlds alike, each a MiG-29A whose left wing parted; one flies its piece, the other's is removed at once:
    // their steps timed in alternating second-long rounds while both fly, the medians compared
    fsim::session::World with(fsim::modes::options("with")), without(fsim::modes::options("without"));
    const auto a = fsim::modes::wing(with, "mig29a", 9000.0, 220.0), b = fsim::modes::wing(without, "mig29a", 9000.0, 220.0);
    REQUIRE(with.separateSection(a, "left wing"));
    REQUIRE(without.separateSection(b, "left wing"));
    for (const auto v : without.vehicleIds())
        if (without.info(v)->role == fsim::ms::Role::Debris) without.removeVehicle(v);
    std::vector<double> tw, to;
    const unsigned n = fsim::modes::stepsFor(with, 1.0);
    auto timed = [&](fsim::session::World& w) {
        const auto t0 = std::chrono::steady_clock::now();
        w.step(n);
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() / (n * static_cast<double>(w.frameSkip()));
    };
    for (int round = 0; round < 9; ++round) tw.push_back(timed(with)), to.push_back(timed(without));
    std::sort(tw.begin(), tw.end());
    std::sort(to.begin(), to.end());
    CHECK(!with.vehicleState(a)->crashed);
    std::printf("a 120 Hz step: %.1f us with its piece, %.1f us without - a piece %.1f us (medians of 9 rounds)\n", 1e6 * tw[4], 1e6 * to[4],
                1e6 * (tw[4] - to[4]));
}

TEST_CASE("every aircraft's pieces load and fall in JSBSim", "[damage]") {
    // hangar's pieces (aircraft/<type>/pieces/<type>-<section>/), each loaded, started at 6 km at 200 m/s and stepped
    // 5 s: it stays finite and slows (a tumbling piece's drag)
    auto ground = std::make_shared<fsim::sim::FlatGround>(0.0);
    int loaded = 0;
    for (const auto& d : std::filesystem::directory_iterator(kAircraft)) {
        const auto dir = d.path() / "pieces";
        if (!std::filesystem::is_directory(dir)) continue;
        for (const auto& p : std::filesystem::directory_iterator(dir)) {
            const std::string name = p.path().filename().string();
            INFO(name);
            fsim::sim::JsbsimModel m(1.0 / 120.0, ground);
            fsim::sim::InitialConditions ic;
            ic.altitudeMslM = 6000.0, ic.airspeedTrueMs = 200.0;
            REQUIRE(m.load(fsim::sim::AircraftSpec{name, FSIM_TEST_JSBSIM_ROOT, dir}, ic));
            fsim::sim::ControlInputs in;
            for (int k = 0; k < 600; ++k) m.step(in);
            fsim::sim::VehicleState s;
            m.state(s);
            CHECK(std::isfinite(s.altitudeMslM));
            CHECK(s.airspeedTrueMs < 200.0);
            ++loaded;
        }
    }
    CHECK(loaded >= 120);
}

TEST_CASE("a hit in the air follows where it came from: a holed tank's fuel is thrown to the far side of the burst, a section it "
          "breaks off away from it - each on top of the aircraft's own motion",
          "[damage][world][impact]") {
    // the same burst mirrored: 3 m off a MiG-29A's left side, then its right, the missile overtaking from behind
    struct Seen {
        int spills = 0, pieces = 0;
        double spillSide = 0.0, spillAlong = 0.0; ///< the fireball's motion from the aircraft's: to its right, along its path (m/s)
        double pieceSide = 0.0;                   ///< the broken-off sections' motion from the aircraft's, to its right, summed
        double own = 0.0;
    };
    auto hitAt = [](double side, double aft, double off, double below) {
        fsim::session::World w(fsim::modes::options(side < 0.0 ? "damage-left" : "damage-right"));
        const auto id = fsim::modes::wing(w, "mig29a", 6000.0, 220.0);
        w.step(fsim::modes::stepsFor(w, 3.0));
        const fsim::sim::VehicleState s = *w.vehicleState(id);
        const double* R = s.rotationBodyToEcef;
        auto ecef = [](const fsim::sim::VehicleState& v) {
            const double* T = v.rotationBodyToEcef;
            return V3{T[0] * v.velocityBodyMs[0] + T[1] * v.velocityBodyMs[1] + T[2] * v.velocityBodyMs[2],
                      T[3] * v.velocityBodyMs[0] + T[4] * v.velocityBodyMs[1] + T[5] * v.velocityBodyMs[2],
                      T[6] * v.velocityBodyMs[0] + T[7] * v.velocityBodyMs[1] + T[8] * v.velocityBodyMs[2]};
        };
        const V3 right{R[1], R[4], R[7]}, ahead{R[0], R[3], R[6]}, own = ecef(s);
        Seen seen;
        seen.own = fsim::ms::damage::length(own);
        const auto events = w.detonate(burstBeside(s, {-aft, side * off, below}, {600.0, -side * 80.0, -30.0}));
        for (const auto& ev : events)
            if (ev.kind == static_cast<std::uint8_t>(fsim::ipc::EngagementKind::Fire) && ev.amountKg > 0.0) {
                const V3 rel = V3{ev.velocityEcef[0], ev.velocityEcef[1], ev.velocityEcef[2]} - own;
                ++seen.spills;
                seen.spillSide += fsim::ms::damage::dot(rel, right);
                seen.spillAlong += fsim::ms::damage::dot(V3{ev.velocityEcef[0], ev.velocityEcef[1], ev.velocityEcef[2]}, ahead);
            }
        const V3 after = ecef(*w.vehicleState(id));
        for (const auto v : w.vehicleIds())
            if (w.info(v)->role == fsim::ms::Role::Debris) {
                ++seen.pieces;
                seen.pieceSide += fsim::ms::damage::dot(ecef(*w.vehicleState(v)) - after, right);
            }
        return seen;
    };
    // (where the fragments go is drawn at random: the first of a few bursts beside it that holes a tank and sets its
    // fuel alight, on each side)
    auto hit = [&](double side) {
        Seen seen;
        for (const double aft : {1.0, 0.0, 2.5, -1.5})
            for (const double off : {3.0, 2.0, 4.0})
                for (const double below : {0.3, -0.4}) {
                    seen = hitAt(side, aft, off, below);
                    if (seen.spills > 0) return seen;
                }
        return seen;
    };
    const Seen left = hit(-1.0), right = hit(1.0);
    if (std::getenv("FSIM_DAMAGE_SHOW"))
        for (const Seen* s : {&left, &right})
            std::printf("spills %d: thrown %.1f m/s to its right, %.1f m/s along its path (it flies %.1f); pieces %d, %.1f m/s to its right\n",
                        s->spills, s->spills ? s->spillSide / s->spills : 0.0, s->spills ? s->spillAlong / s->spills : 0.0, s->own, s->pieces,
                        s->pieceSide);
    // a burst that near holes its tanks from either side
    REQUIRE(left.spills > 0);
    REQUIRE(right.spills > 0);
    // the fuel thrown out goes on with the aircraft - along its path at its speed, near enough - and off to the side
    // away from the burst: to its right from a burst on its left, to its left from one on its right
    CHECK(left.spillSide / left.spills > 5.0);
    CHECK(right.spillSide / right.spills < -5.0);
    CHECK(left.spillAlong / left.spills > 0.8 * left.own);
    CHECK(right.spillAlong / right.spills > 0.8 * right.own);
    // (some tens of metres a second: a few per cent of the fragments' speed, not the fragments' own)
    CHECK(std::abs(left.spillSide / left.spills) < 120.0);
    // a section it breaks off at once leaves away from the burst too (the break's spring alone pushes a wing outboard
    // at 5 m/s: towards a burst on its own side)
    if (left.pieces > 0) CHECK(left.pieceSide > -4.0 * left.pieces);
    if (right.pieces > 0) CHECK(right.pieceSide < 4.0 * right.pieces);
}

TEST_CASE("Native component fire events use model-origin geometry at their captured epoch", "[damage][world][event-frame]") {
    // Reuse the already maintained spill fixture grid. This tests publication
    // coordinates, not whether a new encounter achieves a prescribed effect.
    bool seen=false;
    for (const double aft : {1.0,0.0,2.5,-1.5}) {
        for (const double off : {3.0,2.0,4.0}) {
            for (const double below : {0.3,-0.4}) {
                fsim::session::World world(fsim::modes::options("fire-event-frame"));
                const auto id=fsim::modes::wing(world,"mig29a",6000,220);
                world.step(fsim::modes::stepsFor(world,3));
                const auto before=*world.vehicleState(id);
                double cg[3]; REQUIRE(world.model(id)->cgFromOriginBodyM(cg));
                const auto events=world.detonate(burstBeside(before,{-aft,-off,below},{600,80,-30}));
                const auto* damage=world.damage(id); REQUIRE(damage);
                for (const auto& event:events) {
                    if(event.kind!=static_cast<std::uint8_t>(fsim::ipc::EngagementKind::Fire))continue;
                    const auto& components=damage->airframe().components;
                    const auto component=std::find_if(components.begin(),components.end(),[&](const auto& c){return c.name==event.name;});
                    REQUIRE(component!=components.end());
                    const V3 arm=component->centre-V3{cg[0],cg[1],cg[2]};
                    const auto* r=before.rotationBodyToEcef;
                    const double expected[]={before.positionEcef[0]+r[0]*arm.x+r[1]*arm.y+r[2]*arm.z,
                        before.positionEcef[1]+r[3]*arm.x+r[4]*arm.y+r[5]*arm.z,
                        before.positionEcef[2]+r[6]*arm.x+r[7]*arm.y+r[8]*arm.z};
                    INFO("component=" << event.name << " fixture=" << aft << ',' << off << ',' << below);
                    for(int k=0;k<3;++k)CHECK(std::abs(event.positionEcef[k]-expected[k])<1e-7);
                    seen=true;
                }
                if(seen)break;
            }
            if(seen)break;
        }
        if(seen)break;
    }
    REQUIRE(seen);
}

TEST_CASE("Native fire-rate updates stay on the moving component with its rigid-point velocity", "[damage][world][event-frame]") {
    // Establish a burning tank with the same maintained spill-fixture grid
    // used above. This publication test must not require one particular
    // angular source to ignite a tank at the grid's first point.
    std::unique_ptr<fsim::session::World> fixture;
    std::uint32_t id=0;
    for(const double aft:{1.0,0.0,2.5,-1.5}) {
        for(const double off:{3.0,2.0,4.0}) {
            for(const double below:{0.3,-0.4}) {
                auto candidate=std::make_unique<fsim::session::World>(fsim::modes::options("fire-rate-event-frame"));
                const auto candidateId=fsim::modes::wing(*candidate,"mig29a",6000,220);
                candidate->step(fsim::modes::stepsFor(*candidate,3));
                const auto before=*candidate->vehicleState(candidateId);
                candidate->detonate(burstBeside(before,{-aft,-off,below},{600,80,-30}));
                const auto* damage=candidate->damage(candidateId);REQUIRE(damage);
                for(const auto& [tank,state]:damage->tanks())if(state.burning) {
                    id=candidateId;fixture=std::move(candidate);break;
                }
                if(fixture)break;
            }
            if(fixture)break;
        }
        if(fixture)break;
    }
    REQUIRE(fixture);auto& world=*fixture;
    const auto* damage=world.damage(id);REQUIRE(damage);
    bool burning=false;
    for(const auto& [tank,state]:damage->tanks())if(state.burning) {
        // Synthetic depletion forces an extinguishing rate publication, not a different
        // hit outcome. It also exercises the actual updated native CG.
        auto fuel=world.model(id)->property("propulsion/tank["+std::to_string(tank)+"]/contents-lbs");
        REQUIRE(fuel.valid());fuel.set(0);burning=true;
    }
    REQUIRE(burning);world.takeEngagementEvents();world.step();
    const auto current=*world.vehicleState(id);
    double cg[3];REQUIRE(world.model(id)->cgFromOriginBodyM(cg));
    bool update=false;
    for(const auto& event:world.takeEngagementEvents()) {
        if(event.kind!=static_cast<std::uint8_t>(fsim::ipc::EngagementKind::Fire))continue;
        const auto& components=damage->airframe().components;
        const auto component=std::find_if(components.begin(),components.end(),[&](const auto& c){return c.name==event.name;});
        REQUIRE(component!=components.end());
        const V3 arm=component->centre-V3{cg[0],cg[1],cg[2]};
        const auto* omega=current.angularRateBodyRadS;
        const V3 v{current.velocityBodyMs[0]+omega[1]*arm.z-omega[2]*arm.y,
            current.velocityBodyMs[1]+omega[2]*arm.x-omega[0]*arm.z,
            current.velocityBodyMs[2]+omega[0]*arm.y-omega[1]*arm.x};
        const double armArray[]={arm.x,arm.y,arm.z},velocityArray[]={v.x,v.y,v.z};
        for(int row=0;row<3;++row) {
            double p=current.positionEcef[row],speed=0;
            for(int col=0;col<3;++col) {
                p+=current.rotationBodyToEcef[3*row+col]*armArray[col];
                speed+=current.rotationBodyToEcef[3*row+col]*velocityArray[col];
            }
            CHECK(std::abs(event.positionEcef[row]-p)<1e-7);
            CHECK(std::abs(event.velocityEcef[row]-speed)<1e-6);
        }
        update=true;
    }
    REQUIRE(update);
}

TEST_CASE("Native stream slots distinguish live identity from removed and reused entries", "[damage][world][event-frame]") {
    fsim::session::World world(fsim::modes::options("event-stream-identity"));
    const auto first=fsim::modes::wing(world,"mig29a",6000,220,0);
    const auto second=fsim::modes::wing(world,"mig29a",6000,220,1);
    REQUIRE(world.vehicleStreamSlot(first)==0);REQUIRE(world.vehicleStreamSlot(second)==1);
    REQUIRE(world.removeVehicle(first));CHECK_FALSE(world.vehicleStreamSlot(first));
    const auto replacement=fsim::modes::wing(world,"mig29a",6000,220,2);
    CHECK(replacement!=first);CHECK(world.vehicleStreamSlot(replacement)==0);
    CHECK_FALSE(world.vehicleStreamSlot(first));CHECK_FALSE(world.vehicleStreamSlot(0));
}

TEST_CASE("Taking native Hit events does not consume their recording", "[damage][world][event-frame]") {
    namespace fs=std::filesystem;
    const auto path=fs::temp_directory_path()/("fsim-hit-outbox-"+
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".fsrec");
    fsim::ipc::EngagementEvent nativeHit;
    {
        auto options=fsim::modes::options("recorded-hit-outbox");options.recordPath=path;
        fsim::session::World world(options);
        const auto id=fsim::modes::wing(world,"mig29a",6000,220);
        world.step(fsim::modes::stepsFor(world,3));
        const auto state=*world.vehicleState(id);
        // The existing positive damage fixture, not a replacement showcase.
        const auto events=world.detonate(burstBeside(state,{-1,-4,.5},{600,80,-30}));
        const auto hit=std::find_if(events.begin(),events.end(),[](const auto& event) {
            return event.kind==static_cast<std::uint8_t>(fsim::ipc::EngagementKind::Hit);
        });
        REQUIRE(hit!=events.end());REQUIRE(hit->index>0);nativeHit=*hit;
        const auto outbox=world.takeEngagementEvents();
        CHECK(std::count_if(outbox.begin(),outbox.end(),[](const auto& event) {
            return event.kind==static_cast<std::uint8_t>(fsim::ipc::EngagementKind::Hit);
        })==1);
        CHECK(world.takeEngagementEvents().empty());world.step();
    }
    fsim::ipc::Recording recording;std::string error;
    REQUIRE(recording.load(path,&error));
    unsigned count=0;
    for(const auto& frame:recording.frames())for(const auto& event:frame.events) {
        if(event.kind!=static_cast<std::uint8_t>(fsim::ipc::EngagementKind::Hit))continue;
        CHECK(event.slot==nativeHit.slot);CHECK(event.target==nativeHit.target);
        CHECK(event.index==nativeHit.index);CHECK(event.simTime==nativeHit.simTime);
        CHECK(std::string(event.name)==nativeHit.name);
        CHECK(std::equal(std::begin(event.positionEcef),std::end(event.positionEcef),std::begin(nativeHit.positionEcef)));
        CHECK(std::equal(std::begin(event.velocityEcef),std::end(event.velocityEcef),std::begin(nativeHit.velocityEcef)));
        ++count;
    }
    CHECK(count==1);fs::remove(path);
}

TEST_CASE("a section a burst breaks off is thrown away from the burst: up from one below its wing, down from one above it",
          "[damage][world][impact]") {
    // a MiG-29A's left wing, an AIM-120C bursting close under it and close over it: where the break's spring alone
    // would push the wing straight outboard either time, it now leaves with the blast's and the fragments' push on it
    struct Seen {
        int pieces = 0;
        double down = 0.0, out = 0.0; ///< the pieces' motion from the aircraft's: down, and to its left (m/s), summed
        double momentum = 0.0;        ///< what the world's momentum changed by as they left (kg m/s)
    };
    auto hitAt = [](double below, double span, double aft) {
        fsim::session::World w(fsim::modes::options(below > 0.0 ? "damage-under" : "damage-over"));
        const auto id = fsim::modes::wing(w, "mig29a", 6000.0, 220.0);
        w.step(fsim::modes::stepsFor(w, 3.0));
        const fsim::sim::VehicleState s = *w.vehicleState(id);
        const double* R = s.rotationBodyToEcef;
        auto ecef = [](const fsim::sim::VehicleState& v) {
            const double* T = v.rotationBodyToEcef;
            return V3{T[0] * v.velocityBodyMs[0] + T[1] * v.velocityBodyMs[1] + T[2] * v.velocityBodyMs[2],
                      T[3] * v.velocityBodyMs[0] + T[4] * v.velocityBodyMs[1] + T[5] * v.velocityBodyMs[2],
                      T[6] * v.velocityBodyMs[0] + T[7] * v.velocityBodyMs[1] + T[8] * v.velocityBodyMs[2]};
        };
        const V3 right{R[1], R[4], R[7]}, down{R[2], R[5], R[8]};
        w.detonate(burstBeside(s, {-aft, -span, below}, {600.0, 0.0, 0.0}));
        const V3 after = ecef(*w.vehicleState(id));
        Seen seen;
        for (const auto v : w.vehicleIds())
            if (w.info(v)->role == fsim::ms::Role::Debris) {
                const V3 rel = ecef(*w.vehicleState(v)) - after;
                ++seen.pieces;
                seen.down += fsim::ms::damage::dot(rel, down);
                seen.out -= fsim::ms::damage::dot(rel, right);
            }
        return seen;
    };
    auto hit = [&](double sign) {
        Seen seen;
        for (const double gap : {1.0, 0.7, 1.5})
            for (const double span : {3.0, 2.2, 4.0})
                for (const double aft : {1.5, 0.5, 2.5}) {
                    seen = hitAt(sign * gap, span, aft);
                    if (seen.pieces > 0) return seen;
                }
        return seen;
    };
    const Seen under = hit(1.0), over = hit(-1.0);
    if (std::getenv("FSIM_DAMAGE_SHOW"))
        std::printf("under its wing: %d piece(s), %.1f m/s down, %.1f m/s outboard; over it: %d piece(s), %.1f m/s down, %.1f m/s outboard\n",
                    under.pieces, under.down, under.out, over.pieces, over.down, over.out);
    REQUIRE(under.pieces > 0);
    REQUIRE(over.pieces > 0);
    // away from the burst: up from the one under it, down from the one over it - by more than the spring's 5 m/s
    CHECK(under.down / under.pieces < -5.0);
    CHECK(over.down / over.pieces > 5.0);
    // and no faster than a whole section is thrown
    CHECK(std::abs(under.down / under.pieces) < 61.0);
    CHECK(std::abs(over.down / over.pieces) < 61.0);
}

TEST_CASE("a native departing body retains independent damage and receives a later burst", "[damage][world][scene-effects][detached-effects]") {
    fsim::session::World world(fsim::modes::options("detached-effects"));
    const auto parent=fsim::modes::wing(world,"f16c",6000,220);
    world.step(fsim::modes::stepsFor(world,.5));REQUIRE(world.separateSection(parent,"left wing"));
    std::uint32_t piece=0;
    for(const auto id:world.vehicleIds())if(world.info(id)->role==fsim::ms::Role::Debris)piece=id;
    REQUIRE(piece);REQUIRE(world.damage(piece));
    const auto before=world.damage(piece)->attachedGeometry();REQUIRE_FALSE(before.panelIndices().empty());
    const auto& material=world.damage(piece)->airframe();
    const int sourceSection=material.section("left wing");
    for(const auto index:before.panelIndices())CHECK(material.panels[index].section==sourceSection);
    const auto burst=burstBeside(*world.vehicleState(piece),{0,0,0},{600,0,0});
    const auto events=world.detonate(burst);
    const auto slot=world.vehicleStreamSlot(piece);REQUIRE(slot);
    CHECK(std::any_of(events.begin(),events.end(),[&](const auto& event) {
        return event.slot==*slot&&event.kind==static_cast<std::uint8_t>(fsim::ipc::EngagementKind::Hit)&&event.index>0;
    }));
    world.step(4);REQUIRE(world.damage(piece));CHECK(std::isfinite(world.vehicleState(piece)->altitudeMslM));
    REQUIRE(world.resetVehicle(piece));CHECK(world.damage(piece)==nullptr);
    world.detonate(burstBeside(*world.vehicleState(piece),{0,0,0},{600,0,0}));
    REQUIRE(world.damage(piece));
    const auto restored=world.damage(piece)->attachedGeometry();REQUIRE_FALSE(restored.panelIndices().empty());
    for(const auto index:restored.panelIndices())CHECK(world.damage(piece)->airframe().panels[index].section==sourceSection);
}

TEST_CASE("captured native burst waits for real flight and survives source identity removal", "[damage][world][scene-effects][causal-effects]") {
    fsim::session::World world(fsim::modes::options("causal-burst"));
    const auto victim=fsim::modes::wing(world,"f16c",6000,220);
    fsim::ms::WeaponLaunch release;release.type="jsbsim:aim120c";release.offsetBodyM[1]=30;
    const auto source=world.launchWeapon(victim,release);REQUIRE(source);world.step(2);
    const auto sourceSlot=world.vehicleStreamSlot(source);REQUIRE(sourceSlot);
    auto burst=burstBeside(*world.vehicleState(victim),{0,0,0},{600,0,0});burst.weapon=source;burst.stepFraction=1;
    const auto immediate=world.detonateCapturedV1(burst);
    CHECK(std::none_of(immediate.begin(),immediate.end(),[](const auto& event){return event.kind==static_cast<std::uint8_t>(fsim::ipc::EngagementKind::Hit);}));
    REQUIRE(world.removeVehicle(source));
    fsim::session::VehicleSpec replacement;replacement.name="unrelated replacement";replacement.type="jsbsim:aim120c";
    replacement.initial=world.info(victim)->initial;replacement.initial.longitudeDeg+=.1;
    const auto other=world.createVehicle(replacement);REQUIRE(other);CHECK(world.vehicleStreamSlot(other)==sourceSlot);
    world.takeEngagementEvents();world.step();
    const auto events=world.takeEngagementEvents();const auto victimSlot=world.vehicleStreamSlot(victim);REQUIRE(victimSlot);
    const auto hit=std::find_if(events.begin(),events.end(),[&](const auto& event) {
        return event.slot==*victimSlot&&event.kind==static_cast<std::uint8_t>(fsim::ipc::EngagementKind::Hit)&&event.index>0;
    });
    REQUIRE(hit!=events.end());
    // The wire source is the original release's historical stream slot. Its
    // removed Entry and a new occupant cannot erase or replace that cause.
    CHECK(hit->target==*sourceSlot+1);
    CHECK(hit->simTime==world.simTime());
    CHECK(std::string(hit->name)==burst.store);
    const auto outcomes=world.engagementEvents();
    CHECK(std::any_of(outcomes.begin(),outcomes.end(),[&](const auto& event) {
        return event.kind=="damage"&&event.entity==source&&event.launcher==victim&&event.target==victim&&event.timeS>0;
    }));
    CHECK(std::none_of(outcomes.begin(),outcomes.end(),[&](const auto& event){return event.kind=="damage"&&event.entity==other;}));
    const auto timingStart=std::chrono::steady_clock::now();
    const double before=world.simTime();world.step(fsim::modes::stepsFor(world,1.0));
    const double wall=std::chrono::duration<double>(std::chrono::steady_clock::now()-timingStart).count();
    std::printf("SIM_M03_EFFECTS_TIMING simulated_s=%.9f wall_s=%.9f realtime_fraction=%.9f\n",world.simTime()-before,wall,wall/(world.simTime()-before));
}

TEST_CASE("existing synchronous detonation retains captured fraction and weapon diagnostics", "[damage][world][scene-effects][causal-effects]") {
    fsim::session::World world(fsim::modes::options("synchronous-captured-burst"));
    const auto victim=fsim::modes::wing(world,"f16c",6000,220);
    fsim::ms::WeaponLaunch release;release.type="jsbsim:aim120c";release.offsetBodyM[1]=30;
    const auto source=world.launchWeapon(victim,release);REQUIRE(source);world.step(2);
    auto burst=burstBeside(*world.vehicleState(victim),{0,0,0},{600,0,0});burst.weapon=source;burst.stepFraction=1;
    const auto immediate=world.detonate(burst);
    CHECK(std::any_of(immediate.begin(),immediate.end(),[](const auto& event) {
        return event.kind==static_cast<std::uint8_t>(fsim::ipc::EngagementKind::Hit)&&event.index>0;
    }));
    world.takeEngagementEvents();world.step(2);const auto later=world.takeEngagementEvents();
    CHECK(std::none_of(later.begin(),later.end(),[](const auto& event){return event.kind==static_cast<std::uint8_t>(fsim::ipc::EngagementKind::Hit);}));
}

TEST_CASE("a physical reset restores flight control after native crew loss", "[damage][world][reset]") {
    fsim::session::World fresh(fsim::modes::options("damage-reset-fresh"));
    fsim::session::World restored(fsim::modes::options("damage-reset-restored"));
    const auto baseline=fsim::modes::wing(fresh,"f16c",6000.0,220.0);
    const auto id=fsim::modes::wing(restored,"f16c",6000.0,220.0);
    const auto initial=restored.info(id)->initial;
    REQUIRE(restored.separateSection(id,"nose"));
    const auto events=restored.takeEngagementEvents();
    REQUIRE(std::any_of(events.begin(),events.end(),[](const auto& e) {
        return e.kind==static_cast<std::uint8_t>(fsim::ipc::EngagementKind::CrewLost) && std::string(e.name)=="pilot";
    }));
    REQUIRE(restored.resetVehicle(id,&initial));
    REQUIRE(restored.damage(id)==nullptr);
    CHECK(restored.model(id)->energy().massKg==fresh.model(baseline)->energy().massKg);
    const fsim::control::AttitudeCommand demand{0.1,0.03,fsim::control::kHold,0.7,fsim::control::kHold,220.0};
    REQUIRE(fresh.command(baseline,demand));
    REQUIRE(restored.command(id,demand));
    fresh.step(32);
    restored.step(32);
    const auto& expected=*fresh.vehicleState(baseline);
    const auto& actual=*restored.vehicleState(id);
    REQUIRE_FALSE(actual.crashed);
    REQUIRE_FALSE(actual.diverged);
    CHECK(actual.aileronRad==expected.aileronRad);
    CHECK(actual.elevatorRad==expected.elevatorRad);
    CHECK(actual.rudderRad==expected.rudderRad);
    CHECK(std::memcmp(actual.positionEcef,expected.positionEcef,sizeof actual.positionEcef)==0);
    CHECK(std::memcmp(actual.velocityBodyMs,expected.velocityBodyMs,sizeof actual.velocityBodyMs)==0);
    CHECK(std::memcmp(actual.angularRateBodyRadS,expected.angularRateBodyRadS,sizeof actual.angularRateBodyRadS)==0);
}
TEST_CASE("Added skin damage drag dissipates actual air-relative motion at all attitudes", "[damage][damage-passivity-red]") {
    auto frame=std::make_shared<Airframe>();frame->sections.emplace_back();
    Airframe::Panel panel;panel.section=0;panel.areaM2=2;panel.surface=false;frame->panels.push_back(panel);
    VehicleDamage damage(frame);Impact hit;Impact::PanelHits holes;holes.panel=0;holes.holeAreaM2=.01;hit.panels.push_back(holes);
    fsim::Rng random(17);damage.apply(hit,101325,random);REQUIRE(damage.sections()[0].holeM2>0);
    const double magnitude=kDamagedCd*500*damage.sections()[0].holeM2;
    for(int alpha=-180;alpha<=180;alpha+=5)for(int beta=-80;beta<=80;beta+=20) {
        const double a=alpha*std::acos(-1.)/180,b=beta*std::acos(-1.)/180;
        const V3 direction{std::cos(a)*std::cos(b),std::sin(b),std::sin(a)*std::cos(b)};
        Flight flight;flight.dynamicPressurePa=500;flight.airVelocityBodyMs=direction*100;
        V3 force,moment;damage.aeroLoss(flight,force,moment);INFO(alpha);INFO(beta);
        CHECK(dot(force,flight.airVelocityBodyMs)<=1e-10);
        CHECK(length(force+direction*magnitude)<1e-10);
    }
    Flight forward;forward.dynamicPressurePa=500;forward.airVelocityBodyMs={100,0,0};
    V3 force,moment;damage.aeroLoss(forward,force,moment);
    CHECK(near(force.x,-magnitude,1e-12));CHECK(force.y==0);CHECK(force.z==0);
}
