// Independent analytical finite side-case source checks, not equipment calibration.
#include "simulation/mission_systems/damage/Damage.h"
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
