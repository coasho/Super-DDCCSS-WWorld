// Independent analytical finite side-case source checks, not equipment calibration.
#include "simulation/mission_systems/damage/Damage.h"
#include "simulation/mission_systems/damage/WarheadAdapter.h"
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <string>

namespace {
namespace dmg=fsim::ms::damage;
constexpr double pi=3.14159265358979323846;

fsim::ms::StoreCard sourceCard() {
    fsim::ms::StoreCard card;card.warheadKind="fragmentation";
    const auto parameter=[&](const char* name,double value) {
        auto& p=card.parameters[name];p.value=value;const std::string key=name;
        p.unit=(key=="warhead_kg" || key=="explosive_kg" || key=="warhead_tnt_equivalent_kg") ? "kg" : key=="gurney_kms" ? "km/s" :
               key.find("density_kg_m3")!=std::string::npos ? "kg/m^3" :
               (key=="diameter_m" || key=="length_m" || key.find("warhead_station_")==0) ? "m" : "1";
    };
    parameter("warhead_model_version",1);parameter("warhead_fragment_source",1);
    parameter("warhead_kg",20);parameter("explosive_kg",8);parameter("diameter_m",.2);parameter("length_m",2);
    parameter("gurney_kms",2.6);parameter("warhead_structure_fraction",.15);parameter("warhead_diameter_ratio",.9);
    parameter("warhead_tnt_equivalent_kg",4.5);
    parameter("warhead_case_density_kg_m3",7850);parameter("warhead_explosive_density_kg_m3",1700);
    parameter("warhead_structure_density_kg_m3",2780);parameter("warhead_fragment_cd",1.3);
    parameter("warhead_station_x_m",1);parameter("warhead_station_y_m",0);parameter("warhead_station_z_m",0);
    return card;
}

fsim::ms::StoreCard legacyCard() {
    auto card=sourceCard();
    for(auto i=card.parameters.begin();i!=card.parameters.end();) {
        if(i->first.find("warhead_")==0 && i->first!="warhead_kg") i=card.parameters.erase(i);
        else ++i;
    }
    return card;
}

struct Cylinder {
    double outer,inner,length,caseKg,thickness;
};
Cylinder analyticCylinder() {
    Cylinder c;
    c.outer=.2*.9/2;c.caseKg=(20.-8.)*(1-.15);
    const double volume=8./1700+c.caseKg/7850+(20.-8.)*.15/2780;
    c.length=volume/(pi*c.outer*c.outer);
    c.inner=std::sqrt(c.outer*c.outer-c.caseKg/(7850*pi*c.length));
    c.thickness=c.outer-c.inner;
    return c;
}

struct Launch {dmg::V3 at,direction;std::uint64_t rngAfter;};
Launch analyticLaunch(std::uint64_t seed,dmg::V3 centre) {
    const auto cylinder=analyticCylinder();fsim::Rng rng(seed);
    const double axial=cylinder.length*(rng.uniform()-.5);
    const double phi=2*pi*rng.uniform();
    const double radius=std::sqrt(cylinder.inner*cylinder.inner+rng.uniform()*
        (cylinder.outer*cylinder.outer-cylinder.inner*cylinder.inner));
    // For axis +X, the maintained axis-only ECEF frame has e1=-Y,e2=-Z.
    const dmg::V3 direction{0,-std::cos(phi),-std::sin(phi)};
    return {centre+dmg::V3{axial,0,0}+direction*radius,direction,rng.state()};
}

void checkVector(dmg::V3 actual,dmg::V3 expected,double tolerance=1e-9) {
    CHECK(std::abs(actual.x-expected.x)<tolerance);CHECK(std::abs(actual.y-expected.y)<tolerance);
    CHECK(std::abs(actual.z-expected.z)<tolerance);
}

dmg::Target target() {
    dmg::Target t;t.bodyToEcef={dmg::V3{1,0,0},dmg::V3{0,1,0},dmg::V3{0,0,1}};t.airDensity=0;return t;
}
} // namespace

TEST_CASE("one finite scene source conserves every gram across legacy and versioned construction", "[warhead-model][scene-effects]") {
    for(const auto& card:{legacyCard(),sourceCard()}) {
        const auto model=dmg::warheadOf(card);REQUIRE(model);
        fsim::Rng rng(57);dmg::Burst burst;burst.axis={1,0,0};burst.angularVelocityEcef={0,0,4};
        const auto field=dmg::emitFragmentFieldV1(*model,burst,rng);REQUIRE(field.valid);
        REQUIRE(field.projectiles.size()==static_cast<std::size_t>(model->fragments));
        double mass=0;
        for(const auto& particle:field.projectiles) {
            mass+=particle.massKg;
            CHECK(std::abs(particle.at.x)<=model->lengthM/2);
            CHECK(std::hypot(particle.at.y,particle.at.z)<=model->diameterM/2+1e-14);
            const auto relative=particle.velocity-dmg::cross(burst.angularVelocityEcef,particle.at);
            CHECK(std::abs(dmg::length(relative)-model->speedMs)<1e-9);
        }
        CHECK(std::abs(mass-model->caseKg)<1e-9);
    }
}

TEST_CASE("connected source has shared vertices finite material and explicit breakup", "[warhead-model][scene-effects][connected-rod]") {
    const auto constructed=dmg::warheadOf(legacyCard());REQUIRE(constructed);auto model=*constructed;
    model.kind=dmg::Warhead::Kind::Rod;model.fragments=16;model.lengthM=.5;model.diameterM=.1;
    dmg::Burst burst;burst.axis={1,0,0};burst.angularVelocityEcef={2,1,0};fsim::Rng rng(4);
    const auto ring=dmg::emitFragmentFieldV1(model,burst,rng,true);REQUIRE(ring.valid);
    double mass=0;
    for(std::size_t i=0;i<ring.projectiles.size();++i) {
        const auto& edge=ring.projectiles[i];const auto& neighbor=ring.projectiles[(i+1)%ring.projectiles.size()];
        REQUIRE(edge.segment);CHECK(dmg::length(edge.end-neighbor.at)==0);
        CHECK(dmg::length(edge.endVelocity-neighbor.velocity)==0);CHECK(edge.breakupAgeS>0);mass+=edge.massKg;
        const double breakRadius=.05+model.speedMs*edge.breakupAgeS;
        CHECK(std::abs(2*pi*breakRadius-model.fragments*model.lengthM)<1e-10);
    }
    CHECK(std::abs(mass-model.caseKg)<1e-12);
    model.kind=dmg::Warhead::Kind::Fragmentation;const auto state=rng.state();
    CHECK_FALSE(dmg::emitFragmentFieldV1(model,burst,rng,true).valid);CHECK(rng.state()==state);
}

TEST_CASE("connected ring intercepts its annulus without filling the central gap", "[warhead-model][scene-effects][connected-rod]") {
    const auto constructed=dmg::warheadOf(legacyCard());REQUIRE(constructed);auto model=*constructed;
    model.kind=dmg::Warhead::Kind::Rod;model.fragments=16;model.lengthM=1;model.diameterM=.2;model.speedMs=100;model.tntKg=0;
    dmg::Burst burst;burst.axis={0,0,1};fsim::Rng rng(3);const auto ring=dmg::emitFragmentFieldV1(model,burst,rng,true);REQUIRE(ring.valid);
    dmg::Airframe annulus,centre;dmg::Airframe::Component box;box.half={.02,.02,.02};box.centre={1,0,0};annulus.components.push_back(box);
    box.centre={0,0,0};centre.components.push_back(box);dmg::SceneEnvironmentV1 environment;environment.airDensity=0;environment.maxFlightTimeS=.02;
    const auto result=dmg::assessSceneV1(ring,{{1,dmg::AttachedGeometryView(annulus),target()},{2,dmg::AttachedGeometryView(centre),target()}},environment);
    REQUIRE(result.valid);CHECK_FALSE(result.victims[0].impact.components.empty());CHECK(result.victims[1].impact.components.empty());
    CHECK(result.absorbedMassKg<=model.caseKg);CHECK(std::abs(result.absorbedMassKg+result.remainingMassKg-model.caseKg)<1e-10);
}

TEST_CASE("air-slowed connected material cannot break at an unreached vacuum opening radius", "[warhead-model][scene-effects][connected-rod][effects-quality-boundary]") {
    const auto constructed=dmg::warheadOf(legacyCard());REQUIRE(constructed);auto model=*constructed;
    model.kind=dmg::Warhead::Kind::Rod;model.fragments=16;model.lengthM=.5;model.diameterM=.2;model.speedMs=100;model.fragmentAreaM2=1;model.tntKg=0;
    dmg::Burst burst;burst.axis={0,0,1};fsim::Rng rng(3);const auto ring=dmg::emitFragmentFieldV1(model,burst,rng,true);REQUIRE(ring.valid);
    dmg::SceneEnvironmentV1 environment;environment.airDensity=2;environment.maxFlightTimeS=.02;
    const auto& element=ring.projectiles.front();
    const double drag=environment.airDensity*element.cd*element.areaM2/(2*element.massKg);
    const double propagatedRadius=model.diameterM/2+std::log1p(drag*model.speedMs*environment.maxFlightTimeS)/drag;
    const double availableRadius=model.fragments*model.lengthM/(2*pi);
    REQUIRE(propagatedRadius<availableRadius); // Independent premise, not a selected hit outcome.
    const auto result=dmg::assessSceneV1(ring,{},environment);REQUIRE(result.valid);
    // Each element is slowed so strongly that the available material length
    // cannot yet be reached. The vacuum clock is not the physical opening.
    CHECK(ring.projectiles.front().breakupAgeS<environment.maxFlightTimeS);
    CHECK(result.rodBreakups==0);CHECK_FALSE(result.firstRodBreakAgeS);
}

TEST_CASE("legacy construction refuses unrepresentable fragment counts before integer conversion", "[warhead-model][scene-effects][effects-quality-boundary]") {
    auto card=legacyCard();card.parameters["diameter_m"].value=1e-30;
    CHECK_FALSE(dmg::warheadOf(card));
    card=legacyCard();card.parameters["gurney_kms"].value=std::numeric_limits<double>::max()/2;
    CHECK_FALSE(dmg::warheadOf(card));
}

TEST_CASE("connected-ring provider explicitly admits scene execution without changing the old adapter contract", "[warhead-model][scene-effects][connected-rod]") {
    auto card=legacyCard();card.warheadKind="rod";
    fsim::ms::components::DescriptorV1 descriptor;descriptor.kind="warhead";
    descriptor.implementationId="native.warhead.connected_ring.v1";
    descriptor.availability=fsim::ms::components::AvailabilityV1::Available;descriptor.configurationVersion=1;
    descriptor.basis=fsim::ms::Basis::Assumed;descriptor.note="Independent synthetic explicit topology admission.";
    descriptor.validity="Generic geometry and dispatch, not equipment calibration.";
    for(const char* role:{"warhead_kg","explosive_kg","gurney_kms","diameter_m"}) {
        card.parameters[role].note="Synthetic engineering proxy for independent provider admission, not measured equipment data.";
        descriptor.parameterRefs[role]=role;descriptor.parameters[role]=card.parameters.at(role);
    }
    descriptor.bindings={{"burst_reference","captured_weapon_cg"},{"blast","kingery_bulmash.swisdak_1994.free_air.v1"},
        {"penetration","thor.aluminium_2024_t3.v1"},{"assessment","native.fragment_blast.v1"}};
    std::string error;const auto adapter=dmg::makeWarheadAdapter(card,descriptor,error);INFO(error);REQUIRE(adapter);
    const auto* scene=dynamic_cast<const dmg::WarheadSceneExecutionV2*>(adapter.get());REQUIRE(scene);
    dmg::Burst burst;burst.axis={1,0,0};fsim::Rng rng(4);const auto field=scene->emitScene(burst,rng);
    REQUIRE(field.valid);REQUIRE_FALSE(field.projectiles.empty());CHECK(field.projectiles.front().segment);
    card.warheadKind="fragmentation";CHECK_FALSE(dmg::makeWarheadAdapter(card,descriptor,error));
}

TEST_CASE("invalid drag and overflowing source axes refuse before advancing the physical RNG", "[warhead-model][scene-effects][effects-quality-boundary]") {
    const auto model=dmg::warheadOf(sourceCard());REQUIRE(model);dmg::Burst burst;burst.axis={1,0,0};fsim::Rng rng(77);
    const auto state=rng.state();
    for(const double cd:{0.0,-1.0,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
        auto invalid=*model;invalid.fragmentCd=cd;CHECK_FALSE(dmg::emitFragmentFieldV1(invalid,burst,rng).valid);CHECK(rng.state()==state);
    }
    burst.axis={std::numeric_limits<double>::max()/2,0,0};
    CHECK_FALSE(dmg::emitFragmentFieldV1(*model,burst,rng).valid);CHECK(rng.state()==state);
}

TEST_CASE("warhead v1 requires explicit finite configuration and an in-model source", "[warhead-model]") {
    const auto good=sourceCard();REQUIRE(dmg::warheadOf(good));
    for(const char* name:{"warhead_fragment_source","warhead_kg","explosive_kg","diameter_m","length_m","gurney_kms","warhead_tnt_equivalent_kg",
            "warhead_structure_fraction","warhead_diameter_ratio","warhead_case_density_kg_m3","warhead_explosive_density_kg_m3",
            "warhead_structure_density_kg_m3","warhead_fragment_cd","warhead_station_x_m","warhead_station_y_m","warhead_station_z_m"}) {
        auto missing=good;missing.parameters.erase(name);INFO(name);CHECK_FALSE(dmg::warheadOf(missing));
        auto invalid=good;invalid.parameters[name].value=std::numeric_limits<double>::quiet_NaN();CHECK_FALSE(dmg::warheadOf(invalid));
    }
    for(const auto& invalid:std::vector<std::pair<std::string,double>>{{"warhead_model_version",0},{"warhead_model_version",2},
            {"warhead_model_version",1.5},{"warhead_fragment_source",2},{"warhead_structure_fraction",1},
            {"warhead_diameter_ratio",0},{"warhead_diameter_ratio",1.1},{"warhead_fragment_cd",-1},
            {"warhead_case_density_kg_m3",0},{"explosive_kg",20},{"warhead_tnt_equivalent_kg",0},{"warhead_tnt_equivalent_kg",-1},{"warhead_station_x_m",-.1},
            {"warhead_station_x_m",2.1},{"warhead_station_x_m",.01},{"warhead_station_y_m",.2}}) {
        auto card=good;card.parameters[invalid.first].value=invalid.second;INFO(invalid.first);CHECK_FALSE(dmg::warheadOf(card));
    }
    auto rod=good;rod.warheadKind="rod";CHECK_FALSE(dmg::warheadOf(rod));
}

TEST_CASE("warhead v1 finite annulus and integer fragments conserve the declared case mass", "[warhead-model]") {
    const auto w=dmg::warheadOf(sourceCard());REQUIRE(w);const auto c=analyticCylinder();
    CHECK(w->modelVersion==1);CHECK(w->fragmentSource==dmg::Warhead::FragmentSource::CylindricalSideCase);
    REQUIRE(w->stationStructuralM);checkVector(*w->stationStructuralM,{1,0,0});CHECK(w->fragmentCd==1.3);
    CHECK(std::abs(w->lengthM-c.length)<1e-12);CHECK(std::abs(w->diameterM/2-c.outer)<1e-12);
    CHECK(std::abs(w->caseThicknessM-c.thickness)<1e-12);
    const double actualInner=w->diameterM/2-w->caseThicknessM;
    CHECK(std::abs(7850*pi*w->lengthM*(c.outer*c.outer-actualInner*actualInner)-w->caseKg)<1e-10);
    CHECK(std::abs(w->fragments*w->fragmentKg-w->caseKg)<1e-12);
    const auto count=std::llround(c.caseKg/(7850*c.thickness*c.thickness*c.thickness));
    CHECK(w->fragments==count);
    CHECK(std::abs(w->fragmentAreaM2-1.5*std::pow(w->fragmentKg/7850,2./3.))<1e-12);
    CHECK(w->zones.empty()); // V1 is not a game-sourced zone-weight mixture.
}

TEST_CASE("warhead v1 correlates finite side-case position with radial velocity and real torque", "[warhead-model]") {
    auto configured=dmg::warheadOf(sourceCard());REQUIRE(configured);auto w=*configured;
    w.fragments=1;w.fragmentKg=.006;w.speedMs=300;w.tntKg=0;
    w.zones={{0,0,1}}; // Irrelevant legacy input must not rotate the V1 radial normal into the nose.
    const dmg::V3 centre{3,-4,2};const auto launch=analyticLaunch(42,centre);
    dmg::Airframe frame;dmg::Airframe::Component box;
    box.centre=launch.at+launch.direction*5;box.half={.0001,.0001,.0001};frame.components.push_back(box);
    dmg::Burst burst;burst.at=centre;burst.axis={1,0,0};fsim::Rng rng(42);
    const auto hit=dmg::assess(w,burst,frame,target(),rng);
    CHECK(rng.state()==launch.rngAfter);
    REQUIRE(hit.fragmentsStriking==1);REQUIRE(hit.components.size()==1);
    CHECK(std::abs(hit.components[0].energyJ-270)<1e-9);
    const auto impulse=launch.direction*(w.fragmentKg*w.speedMs);
    checkVector(hit.impulseBody,impulse);
    checkVector(hit.angularImpulseBody,dmg::cross(launch.at,impulse));
    REQUIRE(dmg::length(dmg::cross(launch.at-centre,impulse))>1e-6);
}

TEST_CASE("warhead v1 preserves Galilean relative mechanics and exactly three draws per fragment", "[warhead-model]") {
    auto configured=dmg::warheadOf(sourceCard());REQUIRE(configured);auto w=*configured;
    w.fragments=1;w.fragmentKg=.006;w.speedMs=300;w.tntKg=0;
    const auto launch=analyticLaunch(123,{});
    dmg::Airframe frame;dmg::Airframe::Component box;box.centre=launch.at+launch.direction*5;
    box.half={.0001,.0001,.0001};frame.components.push_back(box);
    dmg::Burst burst;burst.axis={1,0,0};auto t=target();fsim::Rng a(123),b(123);
    const auto first=dmg::assess(w,burst,frame,t,a);
    burst.velocity={1000,-500,250};t.velocity=burst.velocity;
    const auto boosted=dmg::assess(w,burst,frame,t,b);
    REQUIRE(first.fragmentsStriking==1);REQUIRE(boosted.fragmentsStriking==1);
    checkVector(first.impulseBody,boosted.impulseBody);checkVector(first.angularImpulseBody,boosted.angularImpulseBody);
    CHECK(a.state()==b.state());CHECK(a.state()==launch.rngAfter);
    w.fragments=17;fsim::Rng actual(77),expected(77);dmg::Airframe empty;
    dmg::assess(w,burst,empty,t,actual);for(int i=0;i<3*w.fragments;++i)expected.next();
    CHECK(actual.state()==expected.state());
}

TEST_CASE("a finite radial side case cannot manufacture a forward-axis fragment hit", "[warhead-model][fragment-order]") {
    // Synthetic geometry, not AIM-120 equipment inputs. All source points have
    // x <= length/2. A +X speed of 100 and radial speed of 300 put every ray
    // outside this small on-axis target before it reaches the target's front.
    const auto configured=dmg::warheadOf(sourceCard());REQUIRE(configured);
    auto w=*configured;w.fragments=64;w.speedMs=300;w.tntKg=0;
    dmg::Airframe frame;dmg::Airframe::Component box;
    box.centre={2,0,0};box.half={.1,.1,.1};frame.components.push_back(box);
    const double earliest=(box.centre.x-box.half.x-w.lengthM/2)/100;
    REQUIRE(earliest>0);
    REQUIRE(w.caseInnerRadiusM+w.speedMs*earliest>std::hypot(box.half.y,box.half.z));
    dmg::Burst burst;burst.axis={1,0,0};burst.velocity={100,0,0};
    for(const std::uint64_t seed:{0ull,7ull,42ull}) {
        fsim::Rng rng(seed),expected(seed);
        const auto hit=dmg::assess(w,burst,frame,target(),rng);
        CHECK(hit.fragmentsThrown==64);CHECK(hit.fragmentsStriking==0);
        CHECK(hit.components.empty());CHECK(hit.panels.empty());
        CHECK(dmg::length(hit.impulseBody)==0);CHECK(dmg::length(hit.angularImpulseBody)==0);
        for(int i=0;i<3*w.fragments;++i)expected.next();
        CHECK(rng.state()==expected.state());
    }
}

TEST_CASE("warhead v1 source points inherit rigid rotation about the common burst centre", "[warhead-model][warhead-spin]") {
    const auto configured=dmg::warheadOf(sourceCard());REQUIRE(configured);auto w=*configured;
    w.fragments=1;w.fragmentKg=.006;w.speedMs=300;w.tntKg=0;
    const dmg::V3 centre{3,-4,2};const auto launch=analyticLaunch(42,centre);
    const auto offset=launch.at-centre;
    const double radiusSquared=offset.y*offset.y+offset.z*offset.z;
    for(const double spin:{0.,1000.}) {
        INFO(spin);
        const dmg::V3 omega{spin,0,0};
        const auto relativeVelocity=launch.direction*w.speedMs+dmg::cross(omega,offset);
        dmg::Airframe frame;dmg::Airframe::Component box;
        box.centre=launch.at+dmg::normalized(relativeVelocity)*5;box.half={.0001,.0001,.0001};frame.components.push_back(box);
        dmg::Burst burst;burst.at=centre;burst.axis={1,0,0};burst.angularVelocityEcef=omega;
        fsim::Rng rng(42);const auto hit=dmg::assess(w,burst,frame,target(),rng);
        CHECK(rng.state()==launch.rngAfter);
        REQUIRE(hit.fragmentsStriking==1);REQUIRE(hit.components.size()==1);
        // Spin around +X adds a tangential speed orthogonal to radial ejection.
        const double energy=.5*w.fragmentKg*(w.speedMs*w.speedMs+spin*spin*radiusSquared);
        CHECK(std::abs(hit.components[0].energyJ-energy)<1e-9);
        const auto impulse=relativeVelocity*w.fragmentKg;
        checkVector(hit.impulseBody,impulse);
        checkVector(hit.angularImpulseBody,dmg::cross(launch.at,impulse));
    }
    dmg::Burst invalid;invalid.axis={1,0,0};invalid.angularVelocityEcef.x=std::numeric_limits<double>::quiet_NaN();
    fsim::Rng actual(42),unchanged(42);const auto unavailable=dmg::assess(w,invalid,dmg::Airframe{},target(),actual);
    CHECK(unavailable.fragmentsThrown==0);CHECK(actual.state()==unchanged.state());
}

TEST_CASE("warhead v1 blast equivalence is declared independently of Gurney fragment energy", "[warhead-model][warhead-blast]") {
    auto card=sourceCard();const auto original=dmg::warheadOf(card);REQUIRE(original);
    card.parameters["gurney_kms"].value=3.1;
    const auto faster=dmg::warheadOf(card);REQUIRE(faster);
    CHECK(original->tntKg==4.5);CHECK(faster->tntKg==4.5);
    CHECK(std::abs(faster->speedMs/original->speedMs-3.1/2.6)<1e-12);
    CHECK(faster->fragmentKg==original->fragmentKg);CHECK(faster->fragments==original->fragments);
    card.parameters["warhead_tnt_equivalent_kg"].value=6;
    const auto strongerBlast=dmg::warheadOf(card);REQUIRE(strongerBlast);
    CHECK(strongerBlast->tntKg==6);CHECK(strongerBlast->speedMs==faster->speedMs);
}

TEST_CASE("unversioned warhead cards preserve the legacy point-source contract", "[warhead-model]") {
    const auto card=legacyCard();
    const auto w=dmg::warheadOf(card);REQUIRE(w);REQUIRE(w->zones.size()==5);
    CHECK(w->modelVersion==0);CHECK_FALSE(w->stationStructuralM);
    const double oldThickness=w->caseKg/(7850*pi*w->diameterM*w->lengthM);
    CHECK(w->caseThicknessM==oldThickness);
    CHECK(w->fragmentKg==7850*oldThickness*oldThickness*oldThickness);
    auto point=*w;point.fragments=1;point.fragmentKg=.006;point.speedMs=300;point.tntKg=0;point.zones={{0,0,1}};
    dmg::Airframe frame;dmg::Airframe::Component box;box.centre={5,0,0};box.half={.0001,.0001,.0001};frame.components.push_back(box);
    dmg::Burst burst;burst.axis={1,0,0};fsim::Rng a(42),b(42);
    const auto still=dmg::assess(point,burst,frame,target(),a);
    burst.angularVelocityEcef={1000,2000,3000};const auto spun=dmg::assess(point,burst,frame,target(),b);
    REQUIRE(still.components.size()==1);REQUIRE(spun.components.size()==1);
    CHECK(still.components[0].energyJ==spun.components[0].energyJ);
    checkVector(still.impulseBody,spun.impulseBody);CHECK(a.state()==b.state());
}

TEST_CASE("warhead v1 rejects scalar unit mismatches and missing unit declarations", "[warhead-model][warhead-boundary]") {
    const auto good=sourceCard();REQUIRE(dmg::warheadOf(good));
    for(const auto& entry:good.parameters) {
        INFO(entry.first);
        auto missing=good;missing.parameters[entry.first].unit.clear();CHECK_FALSE(dmg::warheadOf(missing));
        auto mismatch=good;mismatch.parameters[entry.first].unit=entry.second.unit=="kg" ? "m" : "kg";
        CHECK_FALSE(dmg::warheadOf(mismatch));
    }
}

TEST_CASE("orphan versioned warhead fields cannot silently select legacy behavior", "[warhead-model][warhead-boundary]") {
    const auto legacy=legacyCard();const auto versioned=sourceCard();REQUIRE(dmg::warheadOf(legacy));
    for(const auto& entry:versioned.parameters) {
        if(entry.first.find("warhead_")!=0 || entry.first=="warhead_kg" || entry.first=="warhead_model_version")continue;
        auto orphan=legacy;orphan.parameters[entry.first]=entry.second;INFO(entry.first);CHECK_FALSE(dmg::warheadOf(orphan));
    }
    auto unknown=legacy;unknown.parameters["warhead_unknown_v1_field"].value=1;CHECK_FALSE(dmg::warheadOf(unknown));
}

TEST_CASE("warhead v1 refuses overflowing or collapsed finite-source geometry", "[warhead-model][warhead-boundary]") {
    const auto good=sourceCard();const double maximum=std::numeric_limits<double>::max(),minimum=std::numeric_limits<double>::min();
    for(const auto& invalid:std::vector<std::pair<std::string,double>>{{"warhead_kg",maximum},{"warhead_case_density_kg_m3",minimum},
            {"diameter_m",maximum},{"diameter_m",minimum},{"gurney_kms",maximum},
            {"warhead_case_density_kg_m3",maximum},{"explosive_kg",0}}) {
        auto card=good;card.parameters[invalid.first].value=invalid.second;INFO(invalid.first);CHECK_FALSE(dmg::warheadOf(card));
    }
    auto noStructure=good;noStructure.parameters["warhead_structure_fraction"].value=0;
    REQUIRE(dmg::warheadOf(noStructure));
}

TEST_CASE("warhead v2 keeps finite mass geometry and the complete pinned seven-zone field", "[warhead-model][warhead-v2]") {
    const auto original=dmg::warheadOf(sourceCard());REQUIRE(original);
    auto card=sourceCard();card.parameters["warhead_model_version"].value=2;card.parameters["warhead_fragment_source"].value=2;
    const auto model=dmg::warheadOf(card);REQUIRE(model);
    CHECK(model->modelVersion==2);REQUIRE(model->zones.size()==7);
    const std::vector<dmg::Zone> expected{{0,20,.0375},{20,60,.075},{60,80,.1},{75,100,.6},
        {100,120,.1},{120,160,.075},{160,180,.0125}};
    double share=0;
    for(std::size_t k=0;k<expected.size();++k) {
        CHECK(model->zones[k].fromDeg==expected[k].fromDeg);CHECK(model->zones[k].toDeg==expected[k].toDeg);
        CHECK(model->zones[k].share==expected[k].share);share+=model->zones[k].share;
    }
    CHECK(std::abs(share-1)<1e-15);
    CHECK(model->caseKg==original->caseKg);CHECK(model->diameterM==original->diameterM);
    CHECK(model->lengthM==original->lengthM);CHECK(model->caseInnerRadiusM==original->caseInnerRadiusM);
    CHECK(model->fragmentKg==original->fragmentKg);CHECK(model->fragmentAreaM2==original->fragmentAreaM2);
    CHECK(model->fragments==original->fragments);CHECK(model->speedMs==original->speedMs);CHECK(model->tntKg==original->tntKg);
    REQUIRE(model->stationStructuralM);REQUIRE(original->stationStructuralM);
    checkVector(*model->stationStructuralM,*original->stationStructuralM);
}

TEST_CASE("warhead v2 finite origins angular sectors and rigid velocities use exactly five draws", "[warhead-model][warhead-v2]") {
    auto card=sourceCard();card.parameters["warhead_model_version"].value=2;card.parameters["warhead_fragment_source"].value=2;
    const auto model=dmg::warheadOf(card);REQUIRE(model);auto w=*model;w.fragments=1;w.tntKg=0;
    const auto cylinder=analyticCylinder();
    const dmg::V3 centre{1,2,3},velocity{11,-2,5},omega{2,3,4},targetVelocity{3,4,-1};
    const std::vector<dmg::Zone> zones{{0,20,.0375},{20,60,.075},{60,80,.1},{75,100,.6},
        {100,120,.1},{120,160,.075},{160,180,.0125}};
    std::vector<bool> visited(zones.size(),false);
    for(std::uint64_t seed=0;seed<512;++seed) {
        fsim::Rng expected(seed);
        const double x=cylinder.length*(expected.uniform()-.5),phi=2*pi*expected.uniform();
        const double radius=std::sqrt(cylinder.inner*cylinder.inner+expected.uniform()*(cylinder.outer*cylinder.outer-cylinder.inner*cylinder.inner));
        const auto pick=expected.uniform();double cumulative=0;std::size_t selected=zones.size()-1;
        for(std::size_t k=0;k<zones.size();++k) {cumulative+=zones[k].share;if(pick<cumulative){selected=k;break;}}
        visited[selected]=true;const auto& zone=zones[selected];
        const double cosine=std::cos(zone.toDeg*pi/180)+(std::cos(zone.fromDeg*pi/180)-std::cos(zone.toDeg*pi/180))*expected.uniform();
        const dmg::V3 radial{0,-std::cos(phi),-std::sin(phi)};
        const auto offset=dmg::V3{x,0,0}+radial*radius;
        const auto direction=dmg::V3{cosine,0,0}+radial*std::sqrt(1-cosine*cosine);
        const auto relative=velocity+dmg::cross(omega,offset)+direction*w.speedMs-targetVelocity;
        dmg::Airframe frame;dmg::Airframe::Component box;
        box.centre=centre+offset+dmg::normalized(relative)*5;box.half={.0001,.0001,.0001};frame.components.push_back(box);
        dmg::Burst burst;burst.at=centre;burst.axis={1,0,0};burst.velocity=velocity;burst.angularVelocityEcef=omega;
        auto moving=target();moving.velocity=targetVelocity;fsim::Rng actual(seed);
        const auto hit=dmg::assess(w,burst,frame,moving,actual);INFO(seed);
        CHECK(actual.state()==expected.state());REQUIRE(hit.fragmentsStriking==1);REQUIRE(hit.components.size()==1);
        CHECK(std::abs(hit.components[0].energyJ-.5*w.fragmentKg*dmg::dot(relative,relative))<1e-8);
        checkVector(hit.impulseBody,relative*w.fragmentKg);
        checkVector(hit.angularImpulseBody,dmg::cross(centre+offset,relative*w.fragmentKg));
    }
    for(const auto seen:visited)CHECK(seen);
}

// Explicit GAME source-role compatibility; synthetic inputs, no equipment fit.
#include "simulation/mission_systems/components/registration/BuiltInProvidersV1.h"
#include <algorithm>
#include <functional>
#include <stdexcept>

namespace {
constexpr const char* authoredFieldId="native.warhead.authored_field.v3";
constexpr const char* authoredRingId="native.warhead.authored_ring.v4";

fsim::ms::StoreCard explicitGameCard(bool ring=false,bool discreteRod=false) {
    auto card=legacyCard();card.warheadKind=(ring||discreteRod)?"rod":"fragmentation";
    const auto previous=dmg::warheadOf(card);
    if(!previous)throw std::runtime_error("synthetic legacy source construction failed");
    const auto& w=*previous;
    const auto put=[&](const std::string& key,double value,const char* unit) {
        fsim::ms::Parameter p;p.value=value;p.unit=unit;p.basis=fsim::ms::Basis::Assumed;
        p.note="Retained synthetic GAME source input; no hardware calibration.";card.parameters[key]=p;
    };
    put("warhead_model_version",ring?4:3,"1");
    put("warhead_case_kg",w.caseKg,"kg");
    put("warhead_structure_kg",w.totalKg-w.explosiveKg-w.caseKg,"kg");
    put("warhead_tnt_equivalent_kg",w.tntKg,"kg");
    put("warhead_case_outer_radius_m",w.diameterM/2,"m");
    put("warhead_case_inner_radius_m",std::max(0.,w.diameterM/2-w.caseThicknessM),"m");
    put("warhead_case_length_m",w.lengthM,"m");put("warhead_fragment_count",w.fragments,"1");
    put("warhead_fragment_area_m2",w.fragmentAreaM2,"m^2");put("warhead_source_speed_ms",w.speedMs,"m/s");
    put("warhead_fragment_cd",w.fragmentCd,"1");put("length_m",2.,"m");
    put("warhead_station_x_m",1.,"m");put("warhead_station_y_m",0.,"m");put("warhead_station_z_m",0.,"m");
    if(!ring) {
        put("warhead_zone_count",static_cast<double>(w.zones.size()),"1");
        for(std::size_t i=0;i<w.zones.size();++i) {
            const auto prefix="warhead_zone_"+std::to_string(i);
            put(prefix+"_from_deg",w.zones[i].fromDeg,"deg");put(prefix+"_to_deg",w.zones[i].toDeg,"deg");
            put(prefix+"_weight",w.zones[i].share,"1");
        }
    }
    for(auto& [key,p]:card.parameters) { (void)key;if(p.note.empty())p.note="Synthetic GAME fixture assumption; not measured hardware."; }
    return card;
}

fsim::ms::components::DescriptorV1 explicitGameDescriptor(const fsim::ms::StoreCard& card,bool ring=false) {
    fsim::ms::components::DescriptorV1 d;d.kind="warhead";d.implementationId=ring?authoredRingId:authoredFieldId;
    d.availability=fsim::ms::components::AvailabilityV1::Available;d.configurationVersion=1;
    d.basis=fsim::ms::Basis::Assumed;d.note="Explicit synthetic GAME input roles.";
    d.validity="Software source-role test only; no equipment accuracy claim.";
    for(const auto& [key,p]:card.parameters)if(key!="gurney_kms") {d.parameterRefs[key]=key;d.parameters[key]=p;}
    d.bindings={{"burst_reference","captured_authored_station_from_nose"},{"blast","kingery_bulmash.swisdak_1994.free_air.v1"},
        {"penetration","thor.aluminium_2024_t3.v1"},{"assessment","native.fragment_blast.v1"}};
    d.bindings[ring?"topology":"angular_profile"]=ring?"native.connected_ring.material_limited.v1":"authored.solid_angle_mixture.v1";
    return d;
}

std::unique_ptr<dmg::WarheadAdapter> explicitGameProvider(const fsim::ms::StoreCard& card,bool ring,std::string& error) {
    const auto registry=fsim::ms::components::builtInProvidersV1();
    const auto d=explicitGameDescriptor(card,ring);fsim::ms::components::ProfileV1 profile;
    const fsim::ms::components::ProviderContextV1 context{card,profile,{},std::nullopt};
    return registry.warhead.create(d,context,error);
}

void identicalGameFields(const dmg::FragmentFieldV1& a,const dmg::FragmentFieldV1& b) {
    REQUIRE(a.valid);REQUIRE(b.valid);REQUIRE(a.projectiles.size()==b.projectiles.size());
    CHECK(a.caseKg==b.caseKg);
    for(std::size_t i=0;i<a.projectiles.size();++i) {
        const auto& p=a.projectiles[i];const auto& q=b.projectiles[i];
        CHECK(p.at.x==q.at.x);CHECK(p.at.y==q.at.y);CHECK(p.at.z==q.at.z);
        CHECK(p.end.x==q.end.x);CHECK(p.end.y==q.end.y);CHECK(p.end.z==q.end.z);
        CHECK(p.velocity.x==q.velocity.x);CHECK(p.velocity.y==q.velocity.y);CHECK(p.velocity.z==q.velocity.z);
        CHECK(p.endVelocity.x==q.endVelocity.x);CHECK(p.endVelocity.y==q.endVelocity.y);CHECK(p.endVelocity.z==q.endVelocity.z);
        CHECK(p.massKg==q.massKg);CHECK(p.areaM2==q.areaM2);CHECK(p.cd==q.cd);CHECK(p.segment==q.segment);
        CHECK(p.expansionPerS==q.expansionPerS);CHECK(p.breakupAgeS==q.breakupAgeS);CHECK(p.materialLengthM==q.materialLengthM);
    }
}
} // namespace

TEST_CASE("explicit GAME field and ring policies are real registered native providers", "[warhead-model][authored-source][authored-source-red]") {
    for(const bool ring:{false,true}) DYNAMIC_SECTION((ring?"connected ring V4":"annular field V3")) {
        INFO(ring);const auto card=explicitGameCard(ring);std::string error;
        const auto provider=explicitGameProvider(card,ring,error);INFO(error);REQUIRE(provider);
        CHECK(provider->implementationId()==(ring?authoredRingId:authoredFieldId));
        CHECK(provider->model().modelVersion==(ring?4u:3u));REQUIRE(provider->model().stationStructuralM);
        CHECK(provider->model().stationStructuralM->x==1.);
        const auto* execution=dynamic_cast<const dmg::WarheadSceneExecutionV2*>(provider.get());REQUIRE(execution);
        dmg::Burst burst;burst.axis={1,0,0};fsim::Rng rng(51);const auto field=execution->emitScene(burst,rng);
        REQUIRE(field.valid);REQUIRE_FALSE(field.projectiles.empty());CHECK(field.projectiles.front().segment==ring);
        if(ring) {CHECK(field.projectiles.front().expansionPerS>0);CHECK(field.projectiles.front().breakupAgeS>0);}
    }
}

TEST_CASE("explicit GAME roles preserve every old field element and RNG at identical data", "[warhead-model][authored-source][authored-source-compatibility]") {
    for(const int topology:{0,1,2}) {
        const bool ring=topology==2;auto oldCard=legacyCard();oldCard.warheadKind=topology?"rod":"fragmentation";
        const auto old=dmg::warheadOf(oldCard);REQUIRE(old);
        const auto card=explicitGameCard(ring,topology==1);std::string error;
        const auto provider=explicitGameProvider(card,ring,error);INFO(error);REQUIRE(provider);
        const auto* execution=dynamic_cast<const dmg::WarheadSceneExecutionV2*>(provider.get());REQUIRE(execution);
        dmg::Burst burst;burst.at={1,2,3};burst.axis={1,2,3};burst.velocity={9,-4,3};burst.angularVelocityEcef={2,-1,3};
        fsim::Rng first(117),second(117);
        const auto before=dmg::emitFragmentFieldV1(*old,burst,first,ring),after=execution->emitScene(burst,second);
        identicalGameFields(before,after);CHECK(first.state()==second.state());CHECK(before.tntKg==after.tntKg);
        auto changed=card;changed.parameters["warhead_tnt_equivalent_kg"].value=0;
        auto noBlast=explicitGameProvider(changed,ring,error);REQUIRE(noBlast);
        const auto* noBlastExecution=dynamic_cast<const dmg::WarheadSceneExecutionV2*>(noBlast.get());REQUIRE(noBlastExecution);
        fsim::Rng third(117);const auto unrelated=noBlastExecution->emitScene(burst,third);
        identicalGameFields(after,unrelated);CHECK(third.state()==second.state());CHECK(unrelated.tntKg==0);
        changed=card;changed.parameters["gurney_kms"].value=99;
        auto noDriver=explicitGameProvider(changed,ring,error);REQUIRE(noDriver);
        CHECK(noDriver->model().speedMs==provider->model().speedMs);CHECK(noDriver->model().tntKg==provider->model().tntKg);
    }
}

TEST_CASE("explicit GAME policy rejects incomplete mixed or out of domain roles atomically", "[warhead-model][authored-source][authored-source-boundary]") {
    for(const bool ring:{false,true}) {
        const auto good=explicitGameCard(ring);std::string error;REQUIRE(explicitGameProvider(good,ring,error));
        for(const auto& entry:explicitGameDescriptor(good,ring).parameters) {
            auto missing=good;missing.parameters.erase(entry.first);INFO(entry.first);CHECK_FALSE(explicitGameProvider(missing,ring,error));
            auto unit=good;unit.parameters[entry.first].unit="wrong";CHECK_FALSE(explicitGameProvider(unit,ring,error));
            auto nan=good;nan.parameters[entry.first].value=std::numeric_limits<double>::quiet_NaN();CHECK_FALSE(explicitGameProvider(nan,ring,error));
        }
        for(const auto& [key,value]:std::vector<std::pair<std::string,double>>{{"warhead_fragment_count",0},{"warhead_fragment_count",1.5},
            {"warhead_fragment_count",1000001},{"warhead_case_kg",-1},{"warhead_structure_kg",-1},
            {"warhead_source_speed_ms",0},{"warhead_fragment_area_m2",0},{"warhead_fragment_cd",0},
            {"warhead_tnt_equivalent_kg",-1},{"warhead_case_outer_radius_m",1},{"warhead_station_x_m",-1}}) {
            auto bad=good;bad.parameters[key].value=value;INFO(key);CHECK_FALSE(explicitGameProvider(bad,ring,error));
        }
        auto extra=good;extra.parameters["warhead_unowned_field"]=good.parameters.at("warhead_case_kg");CHECK_FALSE(explicitGameProvider(extra,ring,error));
        auto mismatch=good;mismatch.parameters["warhead_model_version"].value=ring?3:4;CHECK_FALSE(explicitGameProvider(mismatch,ring,error));
        auto wrongBinding=explicitGameDescriptor(good,ring);wrongBinding.bindings["burst_reference"]="captured_weapon_cg";
        CHECK_FALSE(dmg::makeWarheadAdapter(good,wrongBinding,error));
        if(!ring) {
            for(const auto& [key,value]:std::vector<std::pair<std::string,double>>{{"warhead_zone_count",0},{"warhead_zone_count",33},
                {"warhead_zone_0_from_deg",181},{"warhead_zone_0_to_deg",0},{"warhead_zone_0_weight",-1}}) {
                auto bad=good;bad.parameters[key].value=value;CHECK_FALSE(explicitGameProvider(bad,false,error));
            }
        } else {
            auto unused=good;unused.parameters["warhead_zone_count"]=good.parameters.at("warhead_fragment_count");CHECK_FALSE(explicitGameProvider(unused,true,error));
        }
    }
}

TEST_CASE("new GAME policy direct refusal never consumes the caller source RNG", "[warhead-model][authored-source][authored-source-boundary]") {
    for(const bool ring:{false,true}) {
        const auto card=explicitGameCard(ring);std::string error;const auto provider=explicitGameProvider(card,ring,error);REQUIRE(provider);
        dmg::Burst bad;bad.axis={0,0,0};fsim::Rng rng(93);const auto before=rng.state();
        const auto result=provider->assess(bad,dmg::AttachedGeometryView(dmg::Airframe{}),target(),rng);
        CHECK(result.fragmentsThrown==0);CHECK(rng.state()==before);
        bad.axis={1,0,0};auto refusedTarget=target();refusedTarget.airDensity=std::numeric_limits<double>::quiet_NaN();
        const auto sceneRefused=provider->assess(bad,dmg::AttachedGeometryView(dmg::Airframe{}),refusedTarget,rng);
        CHECK(sceneRefused.fragmentsThrown==0);CHECK(rng.state()==before);
    }
}

namespace {
fsim::ms::StoreCard numericMixtureCard(const std::array<double,3>& weights) {
    auto card=explicitGameCard();
    for(auto it=card.parameters.begin();it!=card.parameters.end();) {
        if(it->first.rfind("warhead_zone_",0)==0)it=card.parameters.erase(it);else ++it;
    }
    auto scalar=card.parameters.at("warhead_fragment_count");scalar.value=3;card.parameters["warhead_zone_count"]=scalar;
    scalar.value=16;card.parameters["warhead_fragment_count"]=scalar;
    const std::array<std::array<double,2>,3> bands{{{{10,20}},{{80,90}},{{160,170}}}};
    for(std::size_t i=0;i<weights.size();++i) {
        const auto prefix="warhead_zone_"+std::to_string(i);scalar.unit="deg";
        scalar.value=bands[i][0];card.parameters[prefix+"_from_deg"]=scalar;
        scalar.value=bands[i][1];card.parameters[prefix+"_to_deg"]=scalar;
        scalar.unit="1";scalar.value=weights[i];card.parameters[prefix+"_weight"]=scalar;
    }
    return card;
}
}

TEST_CASE("authored mixture scaling preserves all particles RNG and zero-weight exclusion", "[warhead-model][authored-source][authored-source-numeric]") {
    const double tiny=std::numeric_limits<double>::denorm_min();
    const std::array<std::array<double,3>,4> weights{{{{1,1,0}},{{tiny,tiny,0}},{{std::scalbn(1.,1023),std::scalbn(1.,1023),0}},
        {{std::numeric_limits<double>::max(),std::numeric_limits<double>::max(),0}}}};
    std::array<std::unique_ptr<dmg::WarheadAdapter>,4> providers;
    for(std::size_t i=0;i<weights.size();++i) {std::string error;providers[i]=explicitGameProvider(numericMixtureCard(weights[i]),false,error);INFO(error);REQUIRE(providers[i]);}
    dmg::Burst burst;burst.axis={1,0,0};std::array<unsigned,2> seen{};
    for(std::uint64_t seed=0;seed<64;++seed) {
        fsim::Rng expected(seed),referenceRng(seed);
        const auto* reference=dynamic_cast<const dmg::WarheadSceneExecutionV2*>(providers[0].get());REQUIRE(reference);
        const auto field=reference->emitScene(burst,referenceRng);REQUIRE(field.valid);
        for(const auto& particle:field.projectiles) {
            (void)expected.uniform();(void)expected.uniform();(void)expected.uniform();
            const bool second=expected.uniform()>=.5;++seen[second?1u:0u];
            const double low=std::cos((second?90.:20.)*pi/180),high=std::cos((second?80.:10.)*pi/180);
            const double cosine=low+(high-low)*expected.uniform();
            // Independent equal-mixture inverse CDF uses u<1/2, never native
            // cumulative sums or a selected target/hit. Trailing zero cannot win.
            CHECK(particle.velocity.x==cosine*providers[0]->model().speedMs);
        }
        CHECK(expected.state()==referenceRng.state());
        for(std::size_t i=1;i<providers.size();++i) {
            const auto* execution=dynamic_cast<const dmg::WarheadSceneExecutionV2*>(providers[i].get());REQUIRE(execution);
            fsim::Rng actual(seed);const auto scaled=execution->emitScene(burst,actual);
            identicalGameFields(field,scaled);CHECK(actual.state()==referenceRng.state());
        }
    }
    CHECK(seen[0]>0);CHECK(seen[1]>0);
}

TEST_CASE("authored policy refuses unrepresentable element mass and ring intrinsic state", "[warhead-model][authored-source][authored-source-numeric]") {
    SECTION("positive case mass cannot become zero per element") {
        auto card=explicitGameCard();card.parameters["warhead_case_kg"].value=std::numeric_limits<double>::denorm_min();
        card.parameters["warhead_fragment_count"].value=1000000;
        card.parameters["warhead_structure_kg"].value=card.parameters.at("warhead_kg").value-card.parameters.at("explosive_kg").value-card.parameters.at("warhead_case_kg").value;
        std::string error;CHECK_FALSE(explicitGameProvider(card,false,error));CHECK(error.find("derived arithmetic")!=std::string::npos);
    }
    SECTION("finite ring source scalars cannot publish infinite expansion") {
        auto card=explicitGameCard(true);card.parameters["warhead_case_outer_radius_m"].value=std::numeric_limits<double>::min();
        card.parameters["warhead_case_inner_radius_m"].value=0;
        std::string error;CHECK_FALSE(explicitGameProvider(card,true,error));CHECK(error.find("ring derived")!=std::string::npos);
    }
    SECTION("actual captured pose cannot publish collapsed ring material") {
        std::string error;const auto provider=explicitGameProvider(explicitGameCard(true),true,error);REQUIRE(provider);
        const auto* execution=dynamic_cast<const dmg::WarheadSceneExecutionV2*>(provider.get());REQUIRE(execution);
        dmg::Burst burst;burst.axis={1,0,0};burst.at={1e100,1e100,1e100};fsim::Rng rng(19);const auto before=rng.state();
        const auto field=execution->emitScene(burst,rng);CHECK_FALSE(field.valid);CHECK(rng.state()==before);
    }
}

TEST_CASE("authored positive mixture weights must remain numerically resolved", "[warhead-model][authored-source][authored-source-numeric]") {
    for(const auto& weights:std::array<std::array<double,3>,3>{{{{std::numeric_limits<double>::max(),std::numeric_limits<double>::denorm_min(),0}},
         {{1,std::numeric_limits<double>::min(),0}},{{std::numeric_limits<double>::min(),1,0}}}}) {
        std::string error;CHECK_FALSE(explicitGameProvider(numericMixtureCard(weights),false,error));
        CHECK(error.find("mixture")!=std::string::npos);
    }
}

// Apply with the separately reviewed19-card authoring, never with production alone.
#include <cwctype>
TEST_CASE("Every authored19 source card reaches its actual equipment factory and population", "[authored-source-models][sim-m03]") {
    const std::array<const char*,19> models{{"aim-120d","aim-7m","aim-9m","aim-9x","asraam","iris-t","magic-2","meteor",
        "mica-em","mica-ir","pl-10","pl-12","pl-15","pl-8","r-37m","r-60m","r-73","r-77m","super-530d"}};
    const auto library=std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR).parent_path()/"assets"/"stores";
    for(const auto& part:library) {
        auto name=part.wstring();std::transform(name.begin(),name.end(),name.begin(),[](wchar_t c){return static_cast<wchar_t>(std::towlower(c));});
        REQUIRE(name!=L".codex");
    }
    const auto registry=fsim::ms::components::builtInProvidersV1();
    for(const char* name:models) DYNAMIC_SECTION(name) {
        INFO(name);std::vector<std::string> errors;const auto path=library/(std::string(name)+".json");
        const auto card=fsim::ms::readStoreCard(path,errors);REQUIRE(errors.empty());REQUIRE(card);
        REQUIRE(card->id==name);CHECK(card->value("warhead_model_version",0)==3);
        fsim::ms::components::ProfileV1 profile;std::string error;
        REQUIRE(fsim::ms::components::readProfileV1(path,*card,profile,error));
        const auto* descriptor=profile.find("warhead");REQUIRE(descriptor);
        CHECK(descriptor->implementationId.rfind("equipment.",0)==0);
        CHECK(descriptor->parameterRefs.count("gurney_kms")==0);
        const fsim::ms::components::ProviderContextV1 context{*card,profile,{},std::nullopt};
        const auto provider=registry.warhead.create(*descriptor,context,error);INFO(error);REQUIRE(provider);
        CHECK(provider->implementationId()==authoredFieldId);const auto& model=provider->model();CHECK(model.modelVersion==3);
        REQUIRE(model.stationStructuralM);CHECK(model.stationStructuralM->x==card->value("warhead_station_x_m",-1));
        CHECK(model.fragments==static_cast<int>(card->value("warhead_fragment_count",-1)));
        CHECK(model.caseKg==card->value("warhead_case_kg",-1));CHECK(model.speedMs==card->value("warhead_source_speed_ms",-1));
        const auto* scene=dynamic_cast<const dmg::WarheadSceneExecutionV2*>(provider.get());REQUIRE(scene);
        dmg::Burst burst;burst.axis={1,0,0};fsim::Rng rng(317);const auto before=rng.state();
        const auto field=scene->emitScene(burst,rng);REQUIRE(field.valid);CHECK(rng.state()!=before);
        REQUIRE(field.projectiles.size()==static_cast<std::size_t>(model.fragments));
        for(const auto& p:field.projectiles) {
            CHECK(p.massKg==model.caseKg/model.fragments);CHECK(p.massKg>0);
            CHECK(p.areaM2==model.fragmentAreaM2);CHECK(p.cd==model.fragmentCd);
            CHECK(p.segment==(card->warheadKind=="rod"));CHECK(p.expansionPerS==0);
        }
    }
}


// Apply only with the separately reviewed four connected-source cards.
TEST_CASE("Every authored ring card retains its connected native population and affine moments", "[authored-ring-models][sim-m03]") {
    struct Expected {const char* name;double station,outer,length,area;};
    const std::array<Expected,4> models{{
        {"r-27er",2.7620237078306715,.117,.2957131060842457,.003606496311644452},
        {"r-27et",2.476909743434796,.117,.2957131060842457,.003606496311644452},
        {"r-27r",1.7677429587802944,.10350000000000001,.3778866913288282,.004052051178232661},
        {"r-27t",1.5553771901419082,.10350000000000001,.3778866913288282,.004052051178232661}}};
    const auto library=std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR).parent_path()/"assets"/"stores";
    for(const auto& part:library) {
        auto name=part.wstring();std::transform(name.begin(),name.end(),name.begin(),[](wchar_t c){return static_cast<wchar_t>(std::towlower(c));});
        REQUIRE(name!=L".codex");
    }
    const auto registry=fsim::ms::components::builtInProvidersV1();
    for(const auto& expected:models) DYNAMIC_SECTION(expected.name) {
        std::vector<std::string> errors;const auto path=library/(std::string(expected.name)+".json");
        const auto card=fsim::ms::readStoreCard(path,errors);REQUIRE(errors.empty());REQUIRE(card);
        REQUIRE(card->id==expected.name);CHECK(card->warheadKind=="rod");
        fsim::ms::components::ProfileV1 profile;std::string error;
        REQUIRE(fsim::ms::components::readProfileV1(path,*card,profile,error));
        const auto* descriptor=profile.find("warhead");REQUIRE(descriptor);
        CHECK(descriptor->parameterRefs.count("gurney_kms")==0);
        CHECK(descriptor->parameterRefs.count("warhead_zone_count")==0);
        CHECK(descriptor->bindings.at("topology")=="native.connected_ring.material_limited.v1");
        const fsim::ms::components::ProviderContextV1 context{*card,profile,{},std::nullopt};
        const auto provider=registry.warhead.create(*descriptor,context,error);INFO(error);REQUIRE(provider);
        CHECK(provider->implementationId()==authoredRingId);const auto& model=provider->model();
        CHECK(model.modelVersion==4);CHECK(model.zones.empty());REQUIRE(model.stationStructuralM);
        CHECK(model.stationStructuralM->x==expected.station);CHECK(model.stationStructuralM->y==0);CHECK(model.stationStructuralM->z==0);
        CHECK(model.fragments==61);CHECK(model.caseKg==20.4);CHECK(model.speedMs==1400);
        CHECK(model.diameterM/2==expected.outer);CHECK(model.lengthM==expected.length);CHECK(model.fragmentAreaM2==expected.area);
        const auto* scene=dynamic_cast<const dmg::WarheadSceneExecutionV2*>(provider.get());REQUIRE(scene);
        dmg::Burst burst;burst.axis={1,0,0};fsim::Rng rng(317);const auto before=rng.state();
        const auto field=scene->emitScene(burst,rng);REQUIRE(field.valid);CHECK(rng.state()!=before);
        REQUIRE(field.projectiles.size()==61);
        long double mass=0,energy=0;dmg::V3 momentum;
        for(std::size_t i=0;i<field.projectiles.size();++i) {
            const auto& p=field.projectiles[i];const auto& next=field.projectiles[(i+1)%field.projectiles.size()];
            CHECK(p.segment);CHECK(p.at.x==0);CHECK(p.end.x==0);
            CHECK(p.end.x==next.at.x);CHECK(p.end.y==next.at.y);CHECK(p.end.z==next.at.z);
            CHECK(p.endVelocity.x==next.velocity.x);CHECK(p.endVelocity.y==next.velocity.y);CHECK(p.endVelocity.z==next.velocity.z);
            CHECK(p.massKg==20.4/61);CHECK(p.areaM2==expected.area);CHECK(p.cd==1.3);
            CHECK(p.expansionPerS==1400/expected.outer);CHECK(p.breakupAgeS>0);CHECK(std::isfinite(p.materialLengthM));
            const double material=61*expected.length*std::sin(pi/61)/pi;
            CHECK(std::abs(p.materialLengthM-material)<=1e-11*material);
            mass+=p.massKg;
            energy+=static_cast<long double>(p.massKg)/6*(dmg::dot(p.velocity,p.velocity)+dmg::dot(p.velocity,p.endVelocity)+dmg::dot(p.endVelocity,p.endVelocity));
            momentum+=(p.velocity+p.endVelocity)*(p.massKg/2);
        }
        // Independent exact affine-velocity integral, not Gurney inference.
        const long double expectedEnergy=.5L*20.4L*1400*1400*(2+std::cos(2*pi/61))/3;
        CHECK(std::abs(mass-20.4L)<=64*61*std::numeric_limits<double>::epsilon()*20.4L);
        CHECK(std::abs(energy-expectedEnergy)<=1e-10L*expectedEnergy);
        CHECK(dmg::length(momentum)<=128*61*std::numeric_limits<double>::epsilon()*20.4*1400);
    }
}
