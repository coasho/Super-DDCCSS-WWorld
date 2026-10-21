#include "simulation/mission_systems/components/Profile.h"
#include "simulation/mission_systems/components/Bindings.h"
#include "foundation/core/Json.h"
#include <catch2/catch_test_macros.hpp>
#include <limits>

namespace {
using fsim::core::Json;
using namespace fsim::ms::components;
const char* roles[] = {"seeker", "guidance", "autopilot", "allocator", "actuator",
                       "fuze", "warhead", "propulsion", "aerodynamics"};
fsim::ms::StoreCard card() {
    fsim::ms::StoreCard c; c.id="synthetic"; c.model="jsbsim:synthetic";
    c.parameters["range"]={100,"m",10,fsim::ms::Basis::Assumed,"","Synthetic software input, not equipment data."};
    return c;
}
Json document() {
    Json root(Json::Object{}); root.set("id","synthetic"); root.set("model","jsbsim:synthetic");
    Json components(Json::Object{});
    for (const auto* role:roles) {
        Json descriptor(Json::Object{});
        descriptor.set("availability","not_applicable"); descriptor.set("reason","Synthetic test has no such component.");
        components.set(role,std::move(descriptor));
    }
    Json seeker(Json::Object{}); seeker.set("availability","available");
    seeker.set("implementation_id","test.seeker"); seeker.set("configuration_version",1);
    seeker.set("basis","assumed"); seeker.set("note","Parser fixture only."); seeker.set("validity","software-test-only");
    Json refs(Json::Object{}); refs.set("reach_m","range"); seeker.set("parameter_refs",refs);
    Json bindings(Json::Object{}); bindings.set("mode","synthetic"); seeker.set("bindings",bindings);
    components.set("seeker",seeker);
    Json profile(Json::Object{}); profile.set("schema_version",1); profile.set("components",components);
    root.set("component_models",profile); return root;
}
Json changedSeeker(Json root,const char* key,Json value) {
    auto p=root.child("component_models"); auto list=p.child("components"); auto s=list.child("seeker");
    s.set(key,std::move(value)); list.set("seeker",s); p.set("components",list); root.set("component_models",p); return root;
}
}

TEST_CASE("Payload profile resolves frozen values and identifies absent legacy declarations", "[payload-components]") {
    auto c=card(); auto doc=document(); ProfileV1 profile; std::string error;
    REQUIRE(parseProfileV1(doc,c,profile,error)); REQUIRE(profile.schemaVersion==1);
    REQUIRE(profile.components.size()==9); REQUIRE(profile.storeId==c.id);
    double reach=-1; REQUIRE(parameterV1(*profile.find("seeker"),"reach_m","m",reach,error)); REQUIRE(reach==100);
    c.parameters["range"].value=900; doc=changedSeeker(doc,"implementation_id","changed");
    REQUIRE(profile.find("seeker")->parameters.at("reach_m").value==100);
    REQUIRE(profile.find("seeker")->implementationId=="test.seeker");
    REQUIRE(parseProfileV1(Json(Json::Object{}),c,profile,error));
    REQUIRE_FALSE(profile.explicitProfile()); REQUIRE(profile.components.empty());
}

TEST_CASE("Explicit payload errors never become a legacy profile or partially replace a resolved one", "[payload-components]") {
    const auto c=card(); ProfileV1 profile; std::string error;
    REQUIRE(parseProfileV1(document(),c,profile,error));
    auto check=[&](const Json& bad,const fsim::ms::StoreCard& input) {
        REQUIRE_FALSE(parseProfileV1(bad,input,profile,error)); REQUIRE_FALSE(error.empty());
        REQUIRE(profile.schemaVersion==1); REQUIRE(profile.find("seeker")->implementationId=="test.seeker");
    };
    check(changedSeeker(document(),"configuration_version",2),c);
    check(changedSeeker(document(),"availability","missing"),c);
    check(changedSeeker(document(),"implementation_id",""),c);
    check(changedSeeker(document(),"basis","invented"),c);
    check(changedSeeker(document(),"note",""),c);
    check(changedSeeker(document(),"validity",""),c);
    Json refs(Json::Object{}); refs.set("reach_m","absent"); check(changedSeeker(document(),"parameter_refs",refs),c);
    auto badCard=c; badCard.parameters["range"].value=std::numeric_limits<double>::infinity(); check(document(),badCard);
    badCard=c; badCard.parameters["range"].uncertainty=-1; check(document(),badCard);
    badCard=c; badCard.parameters["range"].basis=fsim::ms::Basis::Published; check(document(),badCard);
    auto bad=document(); auto p=bad.child("component_models"); p.set("schema_version",2); bad.set("component_models",p); check(bad,c);
    bad=document(); bad.set("id","other"); check(bad,c);
    bad=document(); p=bad.child("component_models"); auto list=p.child("components");
    list.set("unexpected",Json(Json::Object{})); p.set("components",list); bad.set("component_models",p); check(bad,c);
    check(Json::parse(R"({"component_models":{"schema_version":1,"schema_version":1,"components":{}}})"),c);
}

TEST_CASE("Payload providers reject wrong units and unrecognized configuration slots atomically", "[payload-components]") {
    ProfileV1 profile; std::string error; REQUIRE(parseProfileV1(document(),card(),profile,error));
    const auto& d=*profile.find("seeker"); double value=7;
    REQUIRE_FALSE(parameterV1(d,"reach_m","km",value,error)); REQUIRE(value==7);
    REQUIRE_FALSE(parameterV1(d,"missing","m",value,error)); REQUIRE(value==7);
    REQUIRE(slotsV1(d,{"reach_m"},{"mode"},error));
    REQUIRE_FALSE(slotsV1(d,{"other"},{"mode"},error));
    REQUIRE_FALSE(slotsV1(d,{"reach_m"},{},error));
}

TEST_CASE("A rejected loaded card cannot recover another source's same-ID component profile", "[payload-components]") {
    auto a=std::make_shared<fsim::ms::StoreCard>(card());
    auto b=std::make_shared<fsim::ms::StoreCard>(*a);
    auto profile=std::make_shared<ProfileV1>();std::string error;
    REQUIRE(parseProfileV1(document(),*a,*profile,error));
    BindingsV1 bindings;
    REQUIRE(bindings.bind(a,profile,"library-a",error));
    REQUIRE_FALSE(bindings.bind(b,profile,"library-b",error));
    REQUIRE_FALSE(bindings.find(*b,error));REQUIRE_FALSE(error.empty());
    REQUIRE(bindings.find(*a,error)==profile);
    auto unbound=*a; REQUIRE_FALSE(bindings.find(unbound,error));
}

TEST_CASE("Component-only changes and failed reads remain attached to the actual loaded card object", "[payload-components]") {
    auto a=std::make_shared<fsim::ms::StoreCard>(card());
    auto b=std::make_shared<fsim::ms::StoreCard>(*a);
    auto c=std::make_shared<fsim::ms::StoreCard>(*a);
    auto p=std::make_shared<ProfileV1>();std::string error;REQUIRE(parseProfileV1(document(),*a,*p,error));
    auto changed=std::make_shared<ProfileV1>(*p); changed->components.at("seeker").implementationId="changed.provider";
    BindingsV1 bindings;REQUIRE(bindings.bind(a,p,"same-library",error));
    REQUIRE_FALSE(bindings.bind(b,changed,"same-library",error));REQUIRE_FALSE(bindings.find(*b,error));
    bindings.refuse(c,"same-library","The source could not be read.");
    REQUIRE_FALSE(bindings.find(*c,error));REQUIRE(error=="The source could not be read.");
    auto same=std::make_shared<fsim::ms::StoreCard>(*a);
    REQUIRE(bindings.bind(same,std::make_shared<ProfileV1>(*p),"same-library",error));
    REQUIRE(bindings.find(*same,error));
    std::weak_ptr<const fsim::ms::StoreCard> retained=b;b.reset();REQUIRE_FALSE(retained.expired());
    REQUIRE(bindings.find(*a,error)==p);
}
