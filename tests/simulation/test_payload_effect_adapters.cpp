// Provider dispatch and lifecycle preservation; no equipment calibration.
#include "simulation/mission_systems/damage/WarheadAdapter.h"
#include "simulation/mission_systems/weapon/FuzeAdapter.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <utility>

namespace {
namespace ms = fsim::ms;
namespace dmg = ms::damage;
namespace weapon = ms::weapon;
namespace components = ms::components;

ms::Parameter scalar(double value, const char* unit) {
    ms::Parameter p; p.value=value; p.unit=unit;
    p.note="Synthetic provider preservation fixture, not equipment data.";
    return p;
}
components::DescriptorV1 descriptor(const char* kind, const char* implementation) {
    components::DescriptorV1 d; d.kind=kind; d.implementationId=implementation;
    d.availability=components::AvailabilityV1::Available;
    d.note="Wrap the unchanged native reference implementation.";
    d.validity="Synthetic dispatch and lifecycle preservation only.";
    return d;
}
ms::StoreCard warheadCard(bool versioned) {
    ms::StoreCard c; c.warheadKind="fragmentation";
    const auto put=[&](const char* key, double value, const char* unit) {
        c.parameters.emplace(key,scalar(value,unit));
    };
    put("warhead_kg",20,"kg"); put("explosive_kg",8,"kg");
    put("gurney_kms",2.6,"km/s"); put("diameter_m",.2,"m");
    if (versioned) {
        put("warhead_model_version",1,"1"); put("warhead_fragment_source",1,"1");
        put("warhead_tnt_equivalent_kg",4.5,"kg"); put("length_m",2,"m");
        put("warhead_structure_fraction",.15,"1"); put("warhead_diameter_ratio",.9,"1");
        put("warhead_case_density_kg_m3",7850,"kg/m^3");
        put("warhead_explosive_density_kg_m3",1700,"kg/m^3");
        put("warhead_structure_density_kg_m3",2780,"kg/m^3");
        put("warhead_fragment_cd",1.3,"1");
        put("warhead_station_x_m",1,"m"); put("warhead_station_y_m",0,"m");
        put("warhead_station_z_m",0,"m");
    }
    return c;
}
components::DescriptorV1 warheadDescriptor(const ms::StoreCard& card, bool versioned) {
    auto d=descriptor("warhead",versioned?"native.warhead.side_case.v1":"native.warhead.legacy.v1");
    for (const auto& [key,p]:card.parameters) {
        d.parameterRefs.emplace(key,key); d.parameters.emplace(key,p);
    }
    d.bindings={{"burst_reference",versioned?"captured_authored_station_from_nose":"captured_weapon_cg"},
        {"blast","kingery_bulmash.swisdak_1994.free_air.v1"},
        {"penetration","thor.aluminium_2024_t3.v1"},{"assessment","native.fragment_blast.v1"}};
    return d;
}
components::DescriptorV1 fuzeDescriptor(bool versioned) {
    auto d=descriptor("fuze",versioned?"native.fuze.turning_cpa.v1":"native.fuze.legacy.v1");
    const auto put=[&](const char* role, double value, const char* unit) {
        d.parameters.emplace(role,scalar(value,unit));
        d.parameterRefs.emplace(role,std::string("component_fuze_")+role);
    };
    put("model_version",versioned?1:0,"1"); put("algorithm",versioned?1:0,"1");
    put("arming_s",0,"s"); put("life_s",10,"s"); put("radius_m",2,"m");
    if (versioned) {
        put("delay_s",.3,"s"); put("station_x_m",1,"m");
        put("station_y_m",0,"m"); put("station_z_m",0,"m");
        put("body_length_m",2,"m"); put("body_diameter_m",.2,"m");
    }
    d.bindings={{"sensor_reference",versioned?"captured_authored_station_from_nose":"captured_weapon_cg"},
        {"target_reference","captured_target_cg"},{"contact_reference","captured_authored_nose"},
        {"contact","native.authored_point_contact.v1"},{"history","native.captured_piecewise_linear.v1"},
        {"delay",versioned?"native.captured_fixed_delay.v1":"none"}};
    return d;
}
void sameScalar(double actual, double expected) {
    if (std::isnan(expected)) CHECK(std::isnan(actual)); else CHECK(actual==expected);
}
void sameVector(dmg::V3 a, dmg::V3 b) {
    sameScalar(a.x,b.x); sameScalar(a.y,b.y); sameScalar(a.z,b.z);
}
void sameFuze(const weapon::FuzeResult& a, const weapon::FuzeResult& b) {
    CHECK(a.valid==b.valid); CHECK(a.triggered==b.triggered);
    sameScalar(a.missM,b.missM); sameScalar(a.fraction,b.fraction);
}
void sameDelay(const weapon::FuzeDelayResultV1& a, const weapon::FuzeDelayResultV1& b) {
    CHECK(a.valid==b.valid); CHECK(a.pending==b.pending); CHECK(a.due==b.due);
    CHECK(a.targetId==b.targetId); sameScalar(a.ageS,b.ageS); sameScalar(a.fraction,b.fraction);
}
void sameImpact(const dmg::Impact& a, const dmg::Impact& b) {
    sameScalar(a.missM,b.missM); sameVector(a.fromBody,b.fromBody);
    CHECK(a.fragmentsThrown==b.fragmentsThrown); CHECK(a.fragmentsStriking==b.fragmentsStriking);
    sameVector(a.impulseBody,b.impulseBody); sameVector(a.angularImpulseBody,b.angularImpulseBody);
    REQUIRE(a.components.size()==b.components.size()); REQUIRE(a.panels.size()==b.panels.size());
    for (std::size_t i=0;i<a.components.size();++i) {
        const auto& x=a.components[i]; const auto& y=b.components[i];
        CHECK(x.component==y.component); CHECK(x.hits==y.hits);
        sameScalar(x.energyJ,y.energyJ); sameScalar(x.fastestMs,y.fastestMs); sameScalar(x.areaM2,y.areaM2);
    }
    for (std::size_t i=0;i<a.panels.size();++i) {
        const auto& x=a.panels[i]; const auto& y=b.panels[i];
        CHECK(x.panel==y.panel); CHECK(x.hits==y.hits); CHECK(x.perforations==y.perforations);
        sameScalar(x.holeAreaM2,y.holeAreaM2); sameScalar(x.overpressurePa,y.overpressurePa);
        sameScalar(x.impulsePas,y.impulsePas); sameVector(x.momentumNs,y.momentumNs);
    }
}
dmg::Airframe contactFrame() {
    dmg::Airframe frame; dmg::Airframe::Panel p; p.surface=false;
    p.half={1,1,1}; p.areaM2=24; frame.panels.push_back(p);
    dmg::ContactMesh m;
    m.vertices={{-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},
                {-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}};
    m.triangles={{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},
                 {1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,0,4},{3,4,7}};
    m.triangleNeighbors.assign(m.triangles.size(),std::nullopt);
    std::string error; REQUIRE(dmg::setContactGeometryV1(frame,{m},error)); return frame;
}
} // namespace

TEST_CASE("selected warhead providers preserve native assessment and RNG exactly", "[payload-effect-adapters]") {
    for (int mode=0;mode<3;++mode) {
        const bool versioned=mode==2;
        auto card=warheadCard(versioned); if (mode==1) card.warheadKind="rod";
        const auto d=warheadDescriptor(card,versioned); std::string error;
        auto adapter=dmg::makeWarheadAdapter(card,d,error); INFO(error); REQUIRE(adapter);
        CHECK(adapter->implementationId()==d.implementationId); CHECK(error.empty());
        const auto direct=dmg::warheadOf(card); REQUIRE(direct);
        CHECK(adapter->model().modelVersion==direct->modelVersion);
        CHECK(adapter->model().fragmentSource==direct->fragmentSource);
        CHECK(adapter->model().fragments==direct->fragments);
        CHECK(adapter->model().kind==direct->kind);
        dmg::Airframe frame; dmg::Airframe::Component component;
        component.half={20,20,20}; frame.components.push_back(component);
        auto panel=contactFrame().panels.front(); panel.centre={3,0,0};
        frame.panels.push_back(panel);
        dmg::Burst burst; burst.axis={1,0,0}; burst.velocity={19,3,1}; burst.angularVelocityEcef={20,4,1};
        dmg::Target target; target.bodyToEcef={dmg::V3{1,0,0},dmg::V3{0,1,0},dmg::V3{0,0,1}};
        target.velocity={10,1,2}; target.airDensity=0;
        fsim::Rng selected(913), reference(913);
        const auto actual=adapter->assess(burst,dmg::AttachedGeometryView(frame),target,selected);
        const auto expected=dmg::assess(*direct,burst,dmg::AttachedGeometryView(frame),target,reference);
        REQUIRE(expected.fragmentsThrown>0); REQUIRE(expected.fragmentsStriking>0);
        REQUIRE_FALSE(expected.components.empty()); REQUIRE_FALSE(expected.panels.empty());
        REQUIRE(expected.panels.front().overpressurePa>0); sameImpact(actual,expected);
        CHECK(selected.state()==reference.state());
    }
}

TEST_CASE("warhead provider refuses incomplete mismatched or unavailable configuration", "[payload-effect-adapters]") {
    const auto card=warheadCard(true); const auto good=warheadDescriptor(card,true);
    for (int failure=0;failure<15;++failure) {
        auto d=good;
        switch (failure) {
        case 0: d.implementationId="unknown"; break;
        case 1: d.kind="fuze"; break;
        case 2: d.configurationVersion=2; break;
        case 3: d.availability=components::AvailabilityV1::Unavailable; break;
        case 4: d.availability=components::AvailabilityV1::NotApplicable; break;
        case 5: d.parameters.erase("warhead_kg"); break;
        case 6: d.parameters.at("warhead_kg").unit="g"; break;
        case 7: d.bindings.at("blast")="unknown"; break;
        case 8: d.parameters.at("warhead_kg").value=21; break;
        case 9: d.implementationId="native.warhead.legacy.v1"; break;
        case 10: d.bindings.emplace("unknown","unknown"); break;
        case 11: d.parameterRefs.at("warhead_kg")="missing"; break;
        case 12: d.parameterRefs.erase("warhead_kg"); break;
        case 13: d.bindings.erase("assessment"); break;
        case 14: d.parameters.emplace("unknown",scalar(1,"1")); break;
        }
        std::string error; INFO(failure); CHECK_FALSE(dmg::makeWarheadAdapter(card,d,error)); CHECK_FALSE(error.empty());
        CHECK(card.parameters.at("warhead_kg").value==20);
    }
}

TEST_CASE("fuze selection executes distinct native legacy and explicit CPA laws", "[payload-effect-adapters]") {
    const std::vector<weapon::MotionSample> approaching{{0,{3,1,0}},{1,{1,1,0}}};
    const std::vector<weapon::MotionSample> stationary{{0,{}},{1,{}}};
    for (const bool versioned:{false,true}) {
        const auto d=fuzeDescriptor(versioned); weapon::Spec original;
        original.seeker="fixture"; original.navigationGain=4.25;
        std::string error; auto adapter=weapon::makeFuzeAdapter(d,original,error); INFO(error); REQUIRE(adapter);
        CHECK(adapter->implementationId()==d.implementationId); CHECK(error.empty());
        CHECK(adapter->spec().seeker==original.seeker); CHECK(adapter->spec().navigationGain==4.25);
        CHECK(original.armingS==1); CHECK(original.fuzeM==12); CHECK(original.fuzeModelVersion==0);
        CHECK(adapter->spec().fuzeModelVersion==(versioned?1:0)); CHECK(adapter->spec().fuzeAlgorithm==(versioned?1:0));
        CHECK(adapter->spec().armingS==0); CHECK(adapter->spec().lifeS==10); CHECK(adapter->spec().fuzeM==2);
        CHECK(adapter->spec().fuzeDelayS==(versioned?.3:0));
        weapon::FuzeTurnStateV1 direct;
        const auto expected=versioned?direct.evaluate(approaching,stationary,0,1,adapter->spec(),1,false,7):
            weapon::sweptPathFuze(approaching,stationary,0,1,adapter->spec(),1,false);
        const auto actual=adapter->evaluateProximity(approaching,stationary,0,1,1,false,7);
        REQUIRE(expected.valid); CHECK(expected.triggered==!versioned); sameFuze(actual,expected);
        if (versioned) {
            const std::vector<weapon::MotionSample> opening{{0,{1,1,0}},{1,{3,1,0}}};
            const auto turned=direct.evaluate(opening,stationary,1,1,adapter->spec(),1,false,7);
            REQUIRE(turned.triggered); CHECK(turned.fraction==0);
            sameFuze(adapter->evaluateProximity(opening,stationary,1,1,1,false,7),turned);
            CHECK(adapter->spec().fuzeStationStructuralM->x==1);
        } else {
            CHECK_FALSE(adapter->spec().fuzeStationStructuralM); CHECK_FALSE(adapter->scheduleDelay(.25,7));
        }
    }
}

TEST_CASE("fuze adapter keeps contact delay and transient history lifecycles separate", "[payload-effect-adapters]") {
    auto d=fuzeDescriptor(true); d.parameters.at("arming_s").value=.5;
    std::string error; auto adapter=weapon::makeFuzeAdapter(d,weapon::Spec{},error); INFO(error); REQUIRE(adapter);
    const auto frame=contactFrame(); const dmg::AttachedGeometryView geometry(frame);
    const std::vector<weapon::ContactSample> path{{0,{-2,0,0}},{1,{2,0,0}}};
    const weapon::ContactWindow window{0,1,.5,10,1,false};
    const auto expected=weapon::contactFuze(path,geometry,window);
    const auto actual=adapter->evaluateContact(path,geometry,window);
    REQUIRE(expected.valid); REQUIRE(expected.contact); REQUIRE_FALSE(expected.triggered);
    CHECK(actual.valid==expected.valid); CHECK(actual.contact==expected.contact);
    CHECK(actual.triggered==expected.triggered); CHECK(actual.initialOverlap==expected.initialOverlap);
    CHECK(actual.kind==expected.kind); CHECK(actual.index==expected.index);
    sameScalar(actual.offsetS,expected.offsetS); sameScalar(actual.fraction,expected.fraction);
    sameVector(actual.pointBody,expected.pointBody); sameVector(actual.normalBody,expected.normalBody);
    auto mismatched=window; mismatched.armingS=0;
    CHECK_FALSE(adapter->evaluateContact(path,geometry,mismatched).valid);
    CHECK_FALSE(adapter->contactState().consumed());
    CHECK_FALSE(adapter->commitContact(actual)); CHECK(adapter->contactState().consumed());
    weapon::FuzeDelayStateV1 direct;
    REQUIRE(direct.schedule(1.25,7,adapter->spec())); REQUIRE(adapter->scheduleDelay(1.25,7));
    adapter->resetTurnHistory(); CHECK(adapter->contactState().consumed()); CHECK(adapter->delayState().pending());
    sameDelay(adapter->dueDelay(1,.5,.5,false),direct.due(1,.5,adapter->spec(),.5,false));
    const auto due=adapter->dueDelay(1.5,.5,.5,false);
    const auto reference=direct.due(1.5,.5,adapter->spec(),.5,false);
    REQUIRE(reference.due); sameDelay(due,reference); CHECK(due.targetId==7);
    CHECK_FALSE(adapter->scheduleDelay(2,8)); CHECK_FALSE(adapter->dueDelay(2,.5,.5,false).due);
    adapter->resetRelease(); CHECK_FALSE(adapter->contactState().consumed());
    REQUIRE(adapter->scheduleDelay(2,8)); adapter->cancelDelay();
    CHECK_FALSE(adapter->scheduleDelay(2.1,9));
}

TEST_CASE("fuze adapter validation is explicit atomic and does not upgrade legacy", "[payload-effect-adapters]") {
    const auto good=fuzeDescriptor(true); weapon::Spec original; original.armingS=1.125;
    for (int failure=0;failure<18;++failure) {
        auto d=good;
        switch (failure) {
        case 0: d.implementationId="unknown"; break;
        case 1: d.kind="warhead"; break;
        case 2: d.configurationVersion=2; break;
        case 3: d.availability=components::AvailabilityV1::Unavailable; break;
        case 4: d.availability=components::AvailabilityV1::NotApplicable; break;
        case 5: d.parameters.erase("radius_m"); break;
        case 6: d.parameters.at("delay_s").unit="ms"; break;
        case 7: d.bindings.at("history")="predicted"; break;
        case 8: d.parameters.at("algorithm").value=0; break;
        case 9: d.implementationId="native.fuze.legacy.v1"; break;
        case 10: d.bindings.emplace("unknown","unknown"); break;
        case 11: d.parameters.at("delay_s").value=-1; break;
        case 12: d.parameters.at("station_x_m").value=3; break;
        case 13: d.parameters.at("arming_s").value=9.9; break;
        case 14: d.parameters.at("radius_m").value=std::numeric_limits<double>::quiet_NaN(); break;
        case 15: d.parameterRefs.erase("radius_m"); break;
        case 16: d.bindings.erase("contact"); break;
        case 17: d.parameters.emplace("unknown",scalar(1,"1")); break;
        }
        std::string error; INFO(failure); CHECK_FALSE(weapon::makeFuzeAdapter(d,original,error)); CHECK_FALSE(error.empty());
        CHECK(original.armingS==1.125); CHECK(original.fuzeModelVersion==0); CHECK_FALSE(original.fuzeStationStructuralM);
    }
}
