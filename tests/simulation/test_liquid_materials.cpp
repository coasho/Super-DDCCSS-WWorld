#include "simulation/runtime/world/WorldLiquidMaterials.h"
#include "simulation/mission_systems/damage/State.h"
#include "foundation/core/Json.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <string>
#include <utility>

namespace {
using fsim::core::Json;
using namespace fsim::ms::damage;
using namespace fsim::session;

Json sourceDocument() {
    // Same producer schema/frame/policy and method origin as liquid.py. The
    // spatial cells conserve 800 kg / 800 kg/m3 per native tank; the smaller
    // fuel hit probes deliberately do not define capacity geometry.
    return Json::parse(R"({
      "schema":"fsim.liquid-occupancy.v1", "design":"liquid-fixture",
      "frame":"body_origin_forward_right_down_m",
      "method":{"origin_design_m":[10,20,30]},
      "fluid_policy_v1":{"law":"quadratic_unresolved_tumbling_v1",
        "basis":"assumed","drag_coefficient":0.7,"screening_uncertainty_fraction":1},
      "tanks":[
        {"index":0,"section":"body","capacity_kg":800,"density_kg_m3":800,"anchor_body_m":[0.5,0.5,0.5]},
        {"index":1,"section":"wing","capacity_kg":800,"density_kg_m3":800,"anchor_body_m":[2.5,0.5,0.5]}],
      "cells":[
        {"id":0,"tank":0,"section":"body","low_m":[0,0,0],"high_m":[0.5,1,1],"volume_m3":0.5},
        {"id":1,"tank":0,"section":"body","low_m":[0.5,0,0],"high_m":[1,1,1],"volume_m3":0.5},
        {"id":2,"tank":1,"section":"wing","low_m":[2,0,0],"high_m":[3,1,1],"volume_m3":1}]
    })");
}
Json declaration() {
    return Json::parse(R"({"liquid_geometry_v1":{"file":"liquid-fixture.liquid-v1.json",
      "schema":"fsim.liquid-occupancy.v1","required":true}})");
}
std::shared_ptr<Airframe> sourceAirframe() {
    auto frame=std::make_shared<Airframe>();frame->aircraft="liquid-fixture";frame->originDesign={10,20,30};
    frame->sections.resize(2);frame->sections[0].name="body";frame->sections[1].name="wing";
    for(auto& section:frame->sections)section.hasRoot=true;
    frame->sections[0].tanks.push_back({0,"body fuel",800,{0.5,0.5,0.5}});
    frame->sections[1].tanks.push_back({1,"wing fuel",800,{2.5,0.5,0.5}});
    for(unsigned i=0;i<3;++i) {
        Airframe::Component fuel;fuel.kind="fuel";fuel.shape="box";
        fuel.name="fuel probe "+std::to_string(i);fuel.section=i<2?0:1;fuel.tank=i<2?0:1;
        fuel.fuelKg=800;fuel.centre={i==0?0.25:(i==1?0.75:2.5),0.5,0.5};fuel.half={0.1,0.1,0.1};
        frame->components.push_back(fuel);
    }
    Airframe::Component engine;engine.kind="engine";engine.section=1;engine.centre={2.5,0.5,0.5};
    frame->components.push_back(engine);return frame;
}
void member(Json& doc,const char* array,std::size_t index,const char* key,Json value) {
    auto rows=doc.child(array).asArray();rows.at(index).set(key,std::move(value));doc.set(array,Json(std::move(rows)));
}
void erase(Json& doc,const char* key) {
    auto rows=doc.asObject();rows.erase(std::remove_if(rows.begin(),rows.end(),[&](const auto& row){return row.first==key;}),rows.end());
    doc=Json(std::move(rows));
}
void policy(Json& doc,const char* key,Json value) {
    auto row=doc.child("fluid_policy_v1");row.set(key,std::move(value));doc.set("fluid_policy_v1",std::move(row));
}
struct Documents {
    std::filesystem::path directory;
    Documents() {
        static std::atomic<unsigned> sequence{0};
        // The native gate sets TMP/TEMP to D:/ENV/tmp on Windows. No fixture
        // installs an environment or searches outside its own unique directory.
        directory=std::filesystem::temp_directory_path()/("fsim-liquid-materials-"+
            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+"-"+std::to_string(sequence.fetch_add(1)));
        REQUIRE(std::filesystem::create_directory(directory));
    }
    ~Documents() {
        std::error_code ignored;
        std::filesystem::remove(directory/"liquid-fixture.damage.json",ignored);
        std::filesystem::remove(directory/"liquid-fixture.liquid-v1.json",ignored);
        std::filesystem::remove(directory,ignored);
    }
    void text(const char* name,const std::string& value) const {
        std::ofstream stream(directory/name,std::ios::binary|std::ios::trunc);REQUIRE(stream.good());
        stream<<value;stream.close();REQUIRE(stream.good());
    }
    void damage(const Json& doc) const {text("liquid-fixture.damage.json",doc.dump());}
    void liquid(const Json& doc) const {text("liquid-fixture.liquid-v1.json",doc.dump());}
    std::shared_ptr<const LiquidMaterialDatasetV1> read(const Airframe* frame,bool& declared,std::string& error) const {
        return readLiquidMaterialDatasetV1(directory,"liquid-fixture",frame,declared,error);
    }
};
SceneLiquidControlsV1 controls() {
    SceneLiquidControlsV1 out;out.absoluteVolumeToleranceM3=1e-12;out.relativeVolumeTolerance=1e-12;return out;
}
const LiquidTankStateV1& tank(const SceneLiquidRecipientV1& recipient,int index) {
    const auto found=std::find_if(recipient.tanks.begin(),recipient.tanks.end(),[&](const auto& row){return row.nativeTank==index;});
    REQUIRE(found!=recipient.tanks.end());return *found;
}
double occupiedVolume(const SceneLiquidRecipientV1& recipient,int index) {
    const auto& state=tank(recipient,index);double total=0;
    for(const auto& cell:recipient.cells)if(cell.nativeTank==index) {
        const auto volume=fluid::immersedVolume(cell.bodyBounds,state.freeSurfaceBody);REQUIRE(volume);total+=*volume;
    }
    return total;
}
void near(V3 actual,V3 expected) {
    CHECK(actual.x==Catch::Approx(expected.x).epsilon(0).margin(1e-12));
    CHECK(actual.y==Catch::Approx(expected.y).epsilon(0).margin(1e-12));
    CHECK(actual.z==Catch::Approx(expected.z).epsilon(0).margin(1e-12));
}
}

TEST_CASE("Liquid material source binds conserved cells to native tanks and probes", "[damage][liquid-materials]") {
    Documents files;files.damage(declaration());files.liquid(sourceDocument());const auto frame=sourceAirframe();
    bool declared=false;std::string error="old error";const auto data=files.read(frame.get(),declared,error);
    REQUIRE(data);CHECK(declared);CHECK(error.empty());CHECK(data->design=="liquid-fixture");
    CHECK(data->dragCoefficient==Catch::Approx(0.7));REQUIRE(data->tanks.size()==2);REQUIRE(data->cells.size()==3);
    CHECK(data->tanks[0].index==0);CHECK(data->tanks[0].section==0);CHECK(data->tanks[0].capacityKg==800);
    CHECK(data->tanks[1].index==1);CHECK(data->tanks[1].section==1);CHECK(data->tanks[1].densityKgM3==800);
    near(data->tanks[1].anchorBodyM,{2.5,0.5,0.5});
    for(std::size_t i=0;i<data->cells.size();++i) {
        CHECK(data->cells[i].stableCellId==static_cast<std::uint64_t>(i)+1);
        CHECK(data->cells[i].componentIndex==i);CHECK(data->cells[i].section==(i<2?0:1));
    }
    // Neighboring cells share a face but no positive volume. Probe extents do
    // not replace their capacity bounds or density.
    near(data->cells[0].bodyBounds.low,{0,0,0});near(data->cells[0].bodyBounds.high,{0.5,1,1});
    CHECK(frame->components[0].half.x==0.1);
}

TEST_CASE("Liquid declarations distinguish retained absence from refused current data", "[damage][liquid-materials]") {
    Documents files;const auto frame=sourceAirframe();bool declared=true;std::string error="old error";
    CHECK_FALSE(files.read(frame.get(),declared,error));CHECK_FALSE(declared);CHECK(error.empty());
    files.damage(declaration());CHECK_FALSE(files.read(frame.get(),declared,error));CHECK(declared);CHECK_FALSE(error.empty());
    files.liquid(sourceDocument());REQUIRE(files.read(frame.get(),declared,error));CHECK(error.empty());
    files.text("liquid-fixture.liquid-v1.json","{");
    CHECK_FALSE(files.read(frame.get(),declared,error));CHECK(declared);CHECK_FALSE(error.empty());
    files.liquid(sourceDocument());
    for(const auto& change:std::vector<std::pair<std::string,Json>>{
        {"required",false},{"file","../foreign.liquid-v1.json"},{"schema","other.schema"}}) {
        INFO(change.first);auto doc=declaration();auto reference=doc.child("liquid_geometry_v1");
        reference.set(change.first,change.second);doc.set("liquid_geometry_v1",std::move(reference));files.damage(doc);
        CHECK_FALSE(files.read(frame.get(),declared,error));CHECK(declared);CHECK_FALSE(error.empty());
    }
    // A concrete product present without the declaration still opts in; it is
    // parsed, never ignored as retained legacy absence.
    files.damage(Json(Json::Object{}));REQUIRE(files.read(frame.get(),declared,error));CHECK(declared);CHECK(error.empty());
}

TEST_CASE("Liquid parser rejects malformed identity policy geometry and population", "[damage][liquid-materials]") {
    Documents files;files.damage(declaration());const auto frame=sourceAirframe();
    using Mutation=std::pair<const char*,std::function<void(Json&)>>;
    const std::vector<Mutation> mutations{
        {"schema",[](Json& d){d.set("schema","other");}},
        {"frame",[](Json& d){d.set("frame","ecef");}},
        {"design",[](Json& d){d.set("design","other");}},
        {"missing origin",[](Json& d){erase(d,"method");}},
        {"origin mismatch",[](Json& d){d.set("method",Json::parse(R"({"origin_design_m":[10.0001,20,30]})"));}},
        {"missing policy",[](Json& d){erase(d,"fluid_policy_v1");}},
        {"unknown law",[](Json& d){policy(d,"law","other");}},
        {"unearned calibration",[](Json& d){policy(d,"basis","measured");}},
        {"zero Cd",[](Json& d){policy(d,"drag_coefficient",0);}},
        {"negative Cd",[](Json& d){policy(d,"drag_coefficient",-1);}},
        {"zero uncertainty",[](Json& d){policy(d,"screening_uncertainty_fraction",0);}},
        {"negative density",[](Json& d){member(d,"tanks",0,"density_kg_m3",-800);}},
        {"duplicate tank",[](Json& d){auto rows=d.child("tanks").asArray();rows.push_back(rows.front());d.set("tanks",Json(std::move(rows)));}},
        {"fractional tank",[](Json& d){member(d,"tanks",0,"index",0.5);}},
        {"negative tank",[](Json& d){member(d,"tanks",0,"index",-1);}},
        {"tank bound",[](Json& d){member(d,"tanks",0,"index",65536);}},
        {"nonsequential cell",[](Json& d){member(d,"cells",1,"id",0);}},
        {"unknown cell tank",[](Json& d){member(d,"cells",0,"tank",23);}},
        {"cell owner",[](Json& d){member(d,"cells",0,"section","wing");}},
        {"short vector",[](Json& d){member(d,"cells",0,"low_m",Json::parse("[0,0]"));}},
        {"zero dimension",[](Json& d){member(d,"cells",0,"high_m",Json::parse("[0,1,1]"));}},
        {"declared box volume",[](Json& d){member(d,"cells",0,"volume_m3",0.4);}},
        {"missing capacity volume",[](Json& d){member(d,"cells",2,"high_m",Json::parse("[2.5,1,1]"));member(d,"cells",2,"volume_m3",0.5);}},
        {"same-tank overlap",[](Json& d){member(d,"cells",1,"low_m",Json::parse("[0.25,0,0]"));member(d,"cells",1,"high_m",Json::parse("[0.75,1,1]"));}},
        {"cross-tank overlap",[](Json& d){member(d,"cells",2,"low_m",Json::parse("[0.5,0,0]"));member(d,"cells",2,"high_m",Json::parse("[1.5,1,1]"));}}
    };
    for(const auto& mutation:mutations) {
        INFO(mutation.first);auto doc=sourceDocument();mutation.second(doc);files.liquid(doc);
        bool declared=false;std::string error;CHECK_FALSE(files.read(frame.get(),declared,error));CHECK(declared);CHECK_FALSE(error.empty());
    }
}

TEST_CASE("Liquid source requires complete matching native capacity and ownership", "[damage][liquid-materials]") {
    Documents files;files.damage(declaration());
    for(unsigned fault=0;fault<6;++fault) {
        INFO("native ownership fault="<<fault);auto frame=sourceAirframe();auto doc=sourceDocument();
        if(fault==0)member(doc,"tanks",0,"capacity_kg",799);
        if(fault==1)member(doc,"tanks",0,"section","missing");
        if(fault==2)frame->sections[1].tanks.push_back(frame->sections[0].tanks[0]);
        if(fault==3){auto rows=doc.child("tanks").asArray();rows.pop_back();doc.set("tanks",Json(std::move(rows)));
            auto cells=doc.child("cells").asArray();cells.pop_back();doc.set("cells",Json(std::move(cells)));}
        if(fault==4)frame->components[2].tank=99;
        if(fault==5)frame->sections[1].tanks.clear();
        files.liquid(doc);bool declared=false;std::string error;
        CHECK_FALSE(files.read(frame.get(),declared,error));CHECK(declared);CHECK_FALSE(error.empty());
    }
}

TEST_CASE("Liquid inapplicability requires an explicit reason and no native tanks", "[damage][liquid-materials]") {
    Documents files;files.damage(declaration());auto doc=sourceDocument();doc.set("tanks",Json(Json::Array{}));
    doc.set("cells",Json(Json::Array{}));doc.set("no_liquid_reason","No authored native liquid capacity");
    auto frame=sourceAirframe();bool declared=false;std::string error;files.liquid(doc);
    CHECK_FALSE(files.read(frame.get(),declared,error));CHECK_FALSE(error.empty());
    frame->components.clear();for(auto& section:frame->sections)section.tanks.clear();
    auto noReason=doc;erase(noReason,"no_liquid_reason");files.liquid(noReason);
    CHECK_FALSE(files.read(frame.get(),declared,error));CHECK_FALSE(error.empty());
    files.liquid(doc);const auto data=files.read(frame.get(),declared,error);
    REQUIRE(data);CHECK(declared);CHECK(error.empty());CHECK(data->tanks.empty());CHECK(data->cells.empty());
    const auto frozen=freezeLiquidRecipientV1(*data,AttachedGeometryView(*frame),17,3,{}, {},{0,0,1},controls(),error);
    REQUIRE(frozen);CHECK(frozen->tanks.empty());CHECK(frozen->cells.empty());CHECK(error.empty());
}

TEST_CASE("Liquid freeze uses current half full and empty mass without density smearing", "[damage][liquid-materials]") {
    Documents files;files.damage(declaration());files.liquid(sourceDocument());const auto frame=sourceAirframe();
    bool declared=false;std::string error;const auto data=files.read(frame.get(),declared,error);REQUIRE(data);
    for(const double fraction:{0.,0.5,1.}) {
        INFO("current fraction="<<fraction);std::vector<std::optional<double>> contents{800*fraction,800*fraction};
        const auto frozen=freezeLiquidRecipientV1(*data,AttachedGeometryView(*frame),17,3,contents,{}, {0,0,2},controls(),error);
        REQUIRE(frozen);CHECK(error.empty());CHECK(frozen->identity==17);CHECK(frozen->generation==3);
        REQUIRE(frozen->cells.size()==3);REQUIRE(frozen->tanks.size()==2);
        for(int id:{0,1}) {
            const auto& state=tank(*frozen,id);CHECK(state.densityKgM3==800);CHECK(state.dragCoefficient==Catch::Approx(0.7));
            CHECK(state.actualContentsKg==800*fraction);near(state.freeSurfaceBody.normal,{0,0,1});
            CHECK(state.freeSurfaceBody.offset==Catch::Approx(fraction).epsilon(0).margin(2e-12));
            CHECK(occupiedVolume(*frozen,id)==Catch::Approx(fraction).epsilon(0).margin(2e-12));
        }
        contents[0]=123;CHECK(tank(*frozen,0).actualContentsKg==800*fraction); // Captured value, no borrowed vector.
    }
    const auto diagonal=freezeLiquidRecipientV1(*data,AttachedGeometryView(*frame),17,4,{400,400},{},{1,1,1},controls(),error);
    REQUIRE(diagonal);CHECK(diagonal->generation==4);
    CHECK(tank(*diagonal,0).freeSurfaceBody.offset==Catch::Approx(1.5/std::sqrt(3.)).epsilon(0).margin(2e-12));
    CHECK(tank(*diagonal,1).freeSurfaceBody.offset==Catch::Approx(3.5/std::sqrt(3.)).epsilon(0).margin(2e-12));
    CHECK(occupiedVolume(*diagonal,0)==Catch::Approx(0.5).epsilon(0).margin(2e-12));
    CHECK(occupiedVolume(*diagonal,1)==Catch::Approx(0.5).epsilon(0).margin(2e-12));
}

TEST_CASE("Liquid freeze refuses unavailable invalid overcapacity and unbound captures", "[damage][liquid-materials]") {
    Documents files;files.damage(declaration());files.liquid(sourceDocument());const auto frame=sourceAirframe();
    bool declared=false;std::string error;const auto data=files.read(frame.get(),declared,error);REQUIRE(data);
    const double nan=std::numeric_limits<double>::quiet_NaN(),infinity=std::numeric_limits<double>::infinity();
    for(unsigned fault=0;fault<13;++fault) {
        INFO("capture fault="<<fault);std::vector<std::optional<double>> contents{400,400};
        std::uint64_t identity=17,generation=3;V3 shift{},up{0,0,1};auto numeric=controls();
        if(fault==0)contents.clear();
        if(fault==1)contents.pop_back();
        if(fault==2)contents[1].reset();
        if(fault==3)contents[0]=-1;
        if(fault==4)contents[0]=nan;
        if(fault==5)contents[0]=801;
        if(fault==6)identity=0;
        if(fault==7)generation=0;
        if(fault==8)shift.x=nan;
        if(fault==9)up={};
        if(fault==10)numeric.absoluteVolumeToleranceM3=-1e-9;
        if(fault==11)numeric.relativeVolumeTolerance=-1e-9;
        if(fault==12)numeric.absoluteVolumeToleranceM3=infinity;
        CHECK_FALSE(freezeLiquidRecipientV1(*data,AttachedGeometryView(*frame),identity,generation,contents,shift,up,numeric,error));
        CHECK_FALSE(error.empty());
    }
    // An invalid invocation cannot mutate immutable source cells or poison a
    // later successful capture; success clears the previous refusal reason.
    const auto valid=freezeLiquidRecipientV1(*data,AttachedGeometryView(*frame),17,3,{400,400},{},{0,0,1},controls(),error);
    REQUIRE(valid);CHECK(error.empty());near(data->cells[2].bodyBounds.low,{2,0,0});
}

TEST_CASE("Liquid freeze partitions attached ownership and shifts a detached piece once", "[damage][liquid-materials][detached-effects]") {
    Documents files;files.damage(declaration());files.liquid(sourceDocument());const auto frame=sourceAirframe();
    bool declared=false;std::string error;const auto data=files.read(frame.get(),declared,error);REQUIRE(data);
    VehicleDamage parent(frame);REQUIRE(parent.separate(1));const V3 newOrigin{2,0,0};
    auto piece=parent.detachedState(1,newOrigin);REQUIRE(piece);REQUIRE(piece->setDetachedFuelKg(1,400));
    const auto attached=freezeLiquidRecipientV1(*data,parent.attachedGeometry(),17,3,{400,std::nullopt},{},{0,0,1},controls(),error);
    REQUIRE(attached);CHECK(error.empty());REQUIRE(attached->cells.size()==2);REQUIRE(attached->tanks.size()==1);
    CHECK(attached->tanks[0].nativeTank==0);CHECK(attached->cells[0].stableCellId==1);CHECK(attached->cells[1].stableCellId==2);
    const auto detached=freezeLiquidRecipientV1(*data,piece->attachedGeometry(),29,5,{std::nullopt,piece->detachedFuelKg(1)},
        newOrigin,{0,0,1},controls(),error);
    REQUIRE(detached);CHECK(error.empty());CHECK(detached->identity==29);CHECK(detached->generation==5);
    REQUIRE(detached->tanks.size()==1);CHECK(detached->tanks[0].nativeTank==1);REQUIRE(detached->cells.size()==1);
    const auto& cell=detached->cells[0];CHECK(cell.stableCellId==3);CHECK(cell.componentIndex==2);CHECK(cell.section==1);
    near(cell.bodyBounds.low,{0,0,0});near(cell.bodyBounds.high,{1,1,1});
    near(piece->airframe().components[2].centre,{0.5,0.5,0.5});near(data->cells[2].bodyBounds.low,{2,0,0});
    CHECK(occupiedVolume(*attached,0)==Catch::Approx(0.5).epsilon(0).margin(2e-12));
    CHECK(occupiedVolume(*detached,1)==Catch::Approx(0.5).epsilon(0).margin(2e-12));
    CHECK(tank(*attached,0).actualContentsKg+tank(*detached,1).actualContentsKg==800);
}

TEST_CASE("Liquid native tank IDs remain sparse identities rather than row positions", "[damage][liquid-materials]") {
    Documents files;files.damage(declaration());auto frame=sourceAirframe();auto doc=sourceDocument();
    member(doc,"tanks",1,"index",7);member(doc,"cells",2,"tank",7);
    frame->sections[1].tanks[0].index=7;frame->components[2].tank=7;files.liquid(doc);
    bool declared=false;std::string error;const auto data=files.read(frame.get(),declared,error);REQUIRE(data);
    CHECK_FALSE(freezeLiquidRecipientV1(*data,AttachedGeometryView(*frame),17,3,{400,200},{},{0,0,1},controls(),error));
    std::vector<std::optional<double>> contents(8);contents[0]=400;contents[7]=200;
    const auto frozen=freezeLiquidRecipientV1(*data,AttachedGeometryView(*frame),17,3,contents,{},{0,0,1},controls(),error);
    REQUIRE(frozen);CHECK(error.empty());REQUIRE(frozen->tanks.size()==2);CHECK(tank(*frozen,7).actualContentsKg==200);
    CHECK(occupiedVolume(*frozen,7)==Catch::Approx(0.25).epsilon(0).margin(2e-12));
}
